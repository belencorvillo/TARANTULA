#include "WebServer.h"
#include "Tarantula.h"
#include "MotorController.h"
#include "Config.h"

#include <QTcpServer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QUrl>
#include <QCryptographicHash>
#include <QJsonDocument>
#include <QJsonArray>
#include <QRegularExpression>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979f;
constexpr int   STATE_PERIOD_MS = 33;             // ~30 Hz
constexpr qint64 MAX_HTTP_HEADER = 16 * 1024;
constexpr qint64 MAX_WS_PAYLOAD = 1024 * 1024;
const uint32_t MOTOR_IDS[] = { 11,12,13, 21,22,23, 31,32,33, 41,42,43 };

QString stripPortPrefix(const std::string& p)
{
    QString s = QString::fromStdString(p);
    if (s.startsWith("\\\\.\\")) s = s.mid(4);
    return s;
}

// Puertos COM presentes en el sistema (registro de Windows)
QStringList listSerialPorts()
{
    QStringList ports;
#ifdef _WIN32
    HKEY key;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) == ERROR_SUCCESS) {
        for (DWORD i = 0;; ++i) {
            wchar_t name[256]; DWORD nameLen = 256;
            wchar_t data[64];  DWORD dataLen = sizeof(data);
            DWORD type = 0;
            LONG r = RegEnumValueW(key, i, name, &nameLen, nullptr, &type,
                                   reinterpret_cast<LPBYTE>(data), &dataLen);
            if (r != ERROR_SUCCESS) break;
            if (type == REG_SZ)
                ports << QString::fromWCharArray(data, int(dataLen / sizeof(wchar_t))).remove(QChar('\0'));
        }
        RegCloseKey(key);
    }
#endif
    std::sort(ports.begin(), ports.end(), [](const QString& a, const QString& b) {
        return a.mid(3).toInt() < b.mid(3).toInt();
    });
    return ports;
}

QByteArray contentType(const QString& path)
{
    const QString ext = QFileInfo(path).suffix().toLower();
    if (ext == "html") return "text/html; charset=utf-8";
    if (ext == "js" || ext == "mjs") return "text/javascript; charset=utf-8";
    if (ext == "css") return "text/css; charset=utf-8";
    if (ext == "json") return "application/json";
    if (ext == "glb") return "model/gltf-binary";
    if (ext == "svg") return "image/svg+xml";
    if (ext == "png") return "image/png";
    if (ext == "ico") return "image/x-icon";
    return "application/octet-stream";
}

// Sólo aceptamos WebSockets abiertos desde páginas servidas en este mismo equipo.
// Evita que una web cualquiera abierta en el navegador pueda mandar órdenes al robot.
bool isLocalOrigin(const QByteArray& origin)
{
    if (origin.isEmpty()) return true;  // clientes que no son navegador
    const QString host = QUrl(QString::fromLatin1(origin)).host();
    return host == "localhost" || host == "127.0.0.1" || host == "::1";
}

} // namespace

// ============================================================================

WebServer::WebServer(Tarantula* robot, const QString& webRoot, bool sim, QObject* parent)
    : QObject(parent)
    , sim_(sim)
    , robot_(robot)
    , webRoot_(QDir(webRoot).canonicalPath())
    , currentPort_(stripPortPrefix(Config::SERIAL_PORT))
{
    server_ = new QTcpServer(this);
    connect(server_, &QTcpServer::newConnection, this, &WebServer::onNewConnection);

    stateTimer_ = new QTimer(this);
    stateTimer_->setTimerType(Qt::PreciseTimer);
    connect(stateTimer_, &QTimer::timeout, this, &WebServer::broadcastState);
}

WebServer::~WebServer()
{
    // Al cerrar la aplicación, por si acaso: parar la marcha
    if (robot_ && robot_->isGaitActive()) robot_->stopGait();
}

quint16 WebServer::start(quint16 port)
{
    for (quint16 p = port; p < port + 10; ++p) {
        if (server_->listen(QHostAddress::LocalHost, p)) {
            stateTimer_->start(STATE_PERIOD_MS);
            if (sim_) {
                std::cout << "[Web] MODO SIMULACION (--sim): los motores desconectados se simulan\n";
                simActivateOfflineMotors();
                simTimer_ = new QTimer(this);
                simTimer_->setTimerType(Qt::PreciseTimer);
                connect(simTimer_, &QTimer::timeout, this, &WebServer::simStepTrajectories);
                simTimer_->start(20);
            }
            return p;
        }
    }
    return 0;
}

QString WebServer::url() const
{
    return QString("http://localhost:%1/").arg(server_->serverPort());
}

// ============================================================================
// Conexiones
// ============================================================================

void WebServer::onNewConnection()
{
    while (QTcpSocket* sock = server_->nextPendingConnection()) {
        sock->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        clients_.insert(sock, Client{});
        connect(sock, &QTcpSocket::readyRead, this, &WebServer::onReadyRead);
        connect(sock, &QTcpSocket::disconnected, this, &WebServer::onDisconnected);
    }
}

void WebServer::onDisconnected()
{
    auto* sock = qobject_cast<QTcpSocket*>(sender());
    if (!sock) return;
    auto it = clients_.find(sock);
    if (it != clients_.end()) {
        if (it->isWebSocket) onWsClosed(sock);
        clients_.erase(it);
    }
    sock->deleteLater();
}

void WebServer::onReadyRead()
{
    auto* sock = qobject_cast<QTcpSocket*>(sender());
    if (!sock || !clients_.contains(sock)) return;
    Client& c = clients_[sock];
    c.buffer.append(sock->readAll());

    // Ojo: handleHttp puede cerrar el socket (y borrar `c`), por eso se vuelve a buscar
    if (!c.isWebSocket) handleHttp(sock, c);
    auto it = clients_.find(sock);
    if (it != clients_.end() && it->isWebSocket) processWsFrames(sock, *it);
}

// ============================================================================
// HTTP
// ============================================================================

void WebServer::handleHttp(QTcpSocket* sock, Client& c)
{
    const int end = c.buffer.indexOf("\r\n\r\n");
    if (end < 0) {
        if (c.buffer.size() > MAX_HTTP_HEADER) sendHttpError(sock, 431, "Request Header Fields Too Large");
        return;
    }
    const QByteArray head = c.buffer.left(end);
    c.buffer.remove(0, end + 4);

    const QList<QByteArray> lines = head.split('\n');
    const QList<QByteArray> req = lines.value(0).trimmed().split(' ');
    if (req.size() < 2) { sendHttpError(sock, 400, "Bad Request"); return; }
    const QByteArray method = req[0];
    const QString path = QUrl::fromPercentEncoding(req[1].split('?').first());

    QHash<QByteArray, QByteArray> headers;
    for (int i = 1; i < lines.size(); ++i) {
        const int colon = lines[i].indexOf(':');
        if (colon > 0) headers.insert(lines[i].left(colon).trimmed().toLower(), lines[i].mid(colon + 1).trimmed());
    }

    // ---- Upgrade a WebSocket ----
    if (path == "/ws" && headers.value("upgrade").toLower() == "websocket") {
        if (!isLocalOrigin(headers.value("origin"))) {
            std::cout << "[Web] Conexión WebSocket rechazada desde origen no local: "
                      << headers.value("origin").toStdString() << "\n";
            sendHttpError(sock, 403, "Forbidden");
            return;
        }
        const QByteArray key = headers.value("sec-websocket-key");
        if (key.isEmpty()) { sendHttpError(sock, 400, "Bad Request"); return; }
        const QByteArray accept = QCryptographicHash::hash(
            key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11", QCryptographicHash::Sha1).toBase64();
        sock->write("HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Sec-WebSocket-Accept: " + accept + "\r\n\r\n");
        c.isWebSocket = true;
        onWsOpen(sock);
        return;
    }

    if (method != "GET" && method != "HEAD") { sendHttpError(sock, 405, "Method Not Allowed"); return; }
    serveFile(sock, path, method == "HEAD");
}

void WebServer::serveFile(QTcpSocket* sock, const QString& urlPath, bool headOnly)
{
    QString rel = urlPath;
    if (rel.isEmpty() || rel == "/") rel = "/index.html";
    if (rel.contains("..")) { sendHttpError(sock, 403, "Forbidden"); return; }

    const QString full = QFileInfo(webRoot_ + rel).canonicalFilePath();
    if (full.isEmpty() || !full.startsWith(webRoot_ + "/") || !QFileInfo(full).isFile()) {
        sendHttpError(sock, 404, "Not Found");
        return;
    }
    QFile f(full);
    if (!f.open(QIODevice::ReadOnly)) { sendHttpError(sock, 500, "Internal Server Error"); return; }
    const QByteArray body = f.readAll();

    QByteArray resp = "HTTP/1.1 200 OK\r\n"
                      "Content-Type: " + contentType(full) + "\r\n"
                      "Content-Length: " + QByteArray::number(body.size()) + "\r\n"
                      "Cache-Control: no-cache\r\n"
                      "Connection: close\r\n\r\n";
    sock->write(resp);
    if (!headOnly) sock->write(body);
    sock->disconnectFromHost();
}

void WebServer::sendHttpError(QTcpSocket* sock, int code, const char* text)
{
    const QByteArray body = QByteArray::number(code) + " " + text;
    sock->write("HTTP/1.1 " + body + "\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: " + QByteArray::number(body.size()) + "\r\n"
                "Connection: close\r\n\r\n" + body);
    sock->disconnectFromHost();
}

// ============================================================================
// WebSocket (RFC 6455, lo justo para texto)
// ============================================================================

void WebServer::processWsFrames(QTcpSocket* sock, Client& c)
{
    while (true) {
        const QByteArray& b = c.buffer;
        if (b.size() < 2) return;
        const quint8 b0 = quint8(b[0]), b1 = quint8(b[1]);
        const bool fin = b0 & 0x80;
        const int opcode = b0 & 0x0F;
        const bool masked = b1 & 0x80;
        quint64 len = b1 & 0x7F;
        int pos = 2;
        if (len == 126) {
            if (b.size() < 4) return;
            len = qFromBigEndian<quint16>(reinterpret_cast<const uchar*>(b.constData() + 2));
            pos = 4;
        } else if (len == 127) {
            if (b.size() < 10) return;
            len = qFromBigEndian<quint64>(reinterpret_cast<const uchar*>(b.constData() + 2));
            pos = 10;
        }
        if (len > quint64(MAX_WS_PAYLOAD)) { sock->abort(); return; }
        const int maskPos = pos;
        if (masked) pos += 4;
        if (quint64(b.size()) < pos + len) return;

        QByteArray payload = b.mid(pos, int(len));
        if (masked) {
            const char* m = b.constData() + maskPos;
            for (int i = 0; i < payload.size(); ++i) payload[i] = char(payload[i] ^ m[i % 4]);
        }
        c.buffer.remove(0, pos + int(len));

        switch (opcode) {
        case 0x0: // continuación
            c.fragmented.append(payload);
            if (fin) {
                if (c.fragmentedOpcode == 0x1) onWsMessage(sock, c.fragmented);
                c.fragmented.clear();
            }
            break;
        case 0x1: // texto
            if (fin) onWsMessage(sock, payload);
            else { c.fragmented = payload; c.fragmentedOpcode = 0x1; }
            break;
        case 0x8: // cierre
            sendWs(sock, payload.left(2), 0x8);
            sock->disconnectFromHost();
            return;
        case 0x9: // ping
            sendWs(sock, payload, 0xA);
            break;
        default:  // pong / binario: se ignoran
            break;
        }
        if (!clients_.contains(sock)) return;
    }
}

void WebServer::sendWs(QTcpSocket* sock, const QByteArray& payload, int opcode)
{
    QByteArray frame;
    frame.append(char(0x80 | opcode));
    const quint64 n = quint64(payload.size());
    if (n < 126) {
        frame.append(char(n));
    } else if (n < 65536) {
        frame.append(char(126));
        uchar ext[2]; qToBigEndian<quint16>(quint16(n), ext);
        frame.append(reinterpret_cast<char*>(ext), 2);
    } else {
        frame.append(char(127));
        uchar ext[8]; qToBigEndian<quint64>(n, ext);
        frame.append(reinterpret_cast<char*>(ext), 8);
    }
    frame.append(payload);
    sock->write(frame);
}

void WebServer::sendJson(QTcpSocket* sock, const QJsonObject& obj)
{
    sendWs(sock, QJsonDocument(obj).toJson(QJsonDocument::Compact));
}

void WebServer::onWsOpen(QTcpSocket* sock)
{
    // La pestaña más reciente toma el mando; la anterior pasa a solo lectura.
    QTcpSocket* previous = controller_;
    controller_ = sock;
    if (previous && previous != sock) {
        keyUp_ = keyDown_ = keyLeft_ = keyRight_ = false;
        updateGaitFromKeys();
        sendJson(previous, QJsonObject{ {"type", "role"}, {"control", false} });
    }

    QJsonArray ports;
    for (const QString& p : listSerialPorts()) ports.append(p);
    sendJson(sock, QJsonObject{
        {"type", "hello"},
        {"ports", ports},
        {"port", currentPort_},
        {"gaitType", gaitType_},
        {"body", bodyPoseJson()},
        {"control", true},
    });
    std::cout << "[Web] Interfaz conectada (" << clients_.size() << " conexiones)\n";
}

void WebServer::onWsClosed(QTcpSocket* sock)
{
    if (sock != controller_) return;
    controller_ = nullptr;
    // Seguridad: si se cae la pestaña que manda, soltar el D-pad y parar la marcha
    keyUp_ = keyDown_ = keyLeft_ = keyRight_ = false;
    updateGaitFromKeys();
    std::cout << "[Web] La interfaz de control se ha desconectado. Marcha detenida.\n";
}

void WebServer::onWsMessage(QTcpSocket* sock, const QByteArray& text)
{
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(text, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return;
    const QJsonObject m = doc.object();
    const QString cmd = m.value("cmd").toString();

    QJsonObject result;
    static const QStringList READ_ONLY_CMDS = { "ping", "listPorts", "footPos", "takeControl" };
    if (sock != controller_ && !READ_ONLY_CMDS.contains(cmd)) {
        result = { {"ok", false}, {"msg", "Esta pestaña está en solo lectura: otra pestaña tiene el control."} };
    } else if (cmd == "takeControl") {
        QTcpSocket* previous = controller_;
        controller_ = sock;
        if (previous && previous != sock) {
            keyUp_ = keyDown_ = keyLeft_ = keyRight_ = false;
            updateGaitFromKeys();
            sendJson(previous, QJsonObject{ {"type", "role"}, {"control", false} });
        }
        result = { {"ok", true}, {"control", true} };
    } else {
        result = handleCommand(m);
    }

    // Respondemos si el cliente lo pidió (seq) o si hubo un error que mostrar
    if (m.contains("seq") || !result.value("ok").toBool()) {
        result.insert("type", "result");
        result.insert("cmd", cmd);
        if (m.contains("seq")) result.insert("seq", m.value("seq"));
        sendJson(sock, result);
    }
}

// ============================================================================
// Órdenes (mismo comportamiento que MainWindow.cpp)
// ============================================================================

QJsonObject WebServer::handleCommand(const QJsonObject& m)
{
    const QString cmd = m.value("cmd").toString();
    auto ok = [](bool v = true) { return QJsonObject{ {"ok", v} }; };

    if (cmd == "ping") return ok();

    if (cmd == "standUp" || cmd == "sitDown" || cmd == "disableAll") {
        pendingGaitStart_ = false;
        pendingGaitRetries_ = 0;
    }

    if (cmd == "enableAll") {
        robot_->enableAllLegs();
        if (sim_) simActivateOfflineMotors();
        return ok();
    }
    if (cmd == "disableAll") { robot_->disableAllLegs(); return ok(); }
    if (cmd == "standUp")    { robot_->standUp();        return ok(); }
    if (cmd == "sitDown")    { robot_->sitDown();        return ok(); }

    if (cmd == "gaitType") {
        const int t = m.value("type").toInt();
        if (t < 0 || t > 1) return ok(false);
        gaitType_ = t;
        robot_->setGaitType(t);
        return ok();
    }

    if (cmd == "gaitKeys") {
        keyUp_    = m.value("up").toBool();
        keyDown_  = m.value("down").toBool();
        keyLeft_  = m.value("left").toBool();
        keyRight_ = m.value("right").toBool();
        updateGaitFromKeys();
        return ok();
    }

    if (cmd == "bodyPose") {
        static const char* K[6] = { "x", "y", "z", "roll", "pitch", "yaw" };
        static const int LIM[6] = { 120, 120, 140, 25, 25, 25 };   // límites de los sliders
        int v[6];
        for (int i = 0; i < 6; ++i)
            v[i] = std::clamp(m.value(K[i]).toInt(body_[i]), -LIM[i], LIM[i]);

        constexpr double MM = 0.001, DEG = M_PI / 180.0;
        const bool valid = robot_->setBodyPose(v[0] * MM, v[1] * MM, v[2] * MM,
                                               v[3] * DEG, v[4] * DEG, v[5] * DEG);
        if (valid) std::copy(v, v + 6, body_);
        QJsonObject r = ok(valid);
        r.insert("pose", bodyPoseJson());
        if (!valid) r.insert("msg", "La postura solicitada no es factible o causaría una colisión.");
        return r;
    }

    if (cmd == "resetPose") {
        resetPose();
        return ok();
    }

    if (cmd == "legIK") {
        const int leg = m.value("leg").toInt();
        if (leg < 1 || leg > 4) return ok(false);
        const double x = m.value("x").toDouble() * 0.001;
        const double y = m.value("y").toDouble() * 0.001;
        const double z = m.value("z").toDouble() * 0.001;
        const bool valid = robot_->moveLeg(leg, x, y, z);
        if (valid) legIkNominal_ = (m.value("x").toInt() == 495 && m.value("y").toInt() == 0 && m.value("z").toInt() == 0);
        QJsonObject r = ok(valid);
        if (!valid) r.insert("msg", "El movimiento de la pata no es factible o provocaría una colisión.");
        return r;
    }

    if (cmd == "footPos") {
        const int leg = m.value("leg").toInt();
        if (leg < 1 || leg > 4) return ok(false);
        const Eigen::Vector3d p = robot_->getLeg(leg).getCurrentFootPosition();
        return QJsonObject{
            {"ok", true}, {"leg", leg},
            {"x", int(std::lround(p.x() * 1000.0))},
            {"y", int(std::lround(p.y() * 1000.0))},
            {"z", int(std::lround(p.z() * 1000.0))},
        };
    }

    if (cmd == "allJoints") {
        const QJsonArray pos = m.value("pos").toArray();
        const QJsonArray rig = m.value("rig").toArray();
        if (pos.size() != 3 || rig.size() != 3) return ok(false);
        float p[3]; int r[3];
        for (int i = 0; i < 3; ++i) { p[i] = float(pos[i].toDouble()); r[i] = rig[i].toInt(); }
        Tarantula* robot = robot_;
        std::thread([robot, p0 = p[0], p1 = p[1], p2 = p[2], r0 = r[0], r1 = r[1], r2 = r[2]]() {
            for (int leg = 1; leg <= 4; ++leg) {
                robot->moveLegJoint(leg, 1, p0, r0);
                robot->moveLegJoint(leg, 2, p1, r1);
                robot->moveLegJoint(leg, 3, p2, r2);
            }
        }).detach();
        return ok();
    }

    if (cmd == "enableMotor" || cmd == "disableMotor" || cmd == "moveMotor") {
        const int id = m.value("id").toInt();
        const int leg = id / 10, joint = id % 10;
        if (leg < 1 || leg > 4 || joint < 1 || joint > 3) return ok(false);
        Tarantula* robot = robot_;
        if (cmd == "enableMotor") {
            std::thread([robot, leg, joint]() { robot->getLeg(leg).enableJoint(joint); }).detach();
            if (sim_) simActivateOfflineMotors(id);
        } else if (cmd == "disableMotor") {
            std::thread([robot, leg, joint]() { robot->getLeg(leg).disableJoint(joint); }).detach();
        } else {
            const float pos = float(m.value("pos").toDouble());
            const int rig = m.value("rig").toInt();
            std::thread([robot, leg, joint, pos, rig]() {
                robot->getLeg(leg).moveJoint(joint, pos, rig);
            }).detach();
        }
        return ok();
    }

    if (cmd == "resetLog") {
        if (!g_comm) return ok(false);
        g_comm->resetLogger();
        std::cout << "[CanLogger] Archivo can_log.csv reseteado manualmente por el usuario.\n";
        return ok();
    }

    if (cmd == "listPorts") {
        QJsonArray ports;
        for (const QString& p : listSerialPorts()) ports.append(p);
        return QJsonObject{ {"ok", true}, {"ports", ports} };
    }

    if (cmd == "setPort") {
        const QString port = m.value("port").toString().trimmed().toUpper();
        static const QRegularExpression RE("^COM\\d{1,3}$");
        if (!RE.match(port).hasMatch()) return QJsonObject{ {"ok", false}, {"msg", "Nombre de puerto no válido"} };
        std::cout << "🔌 Solicitando reconexión serial a: " << port.toStdString() << "\n";
        currentPort_ = port;
        const bool connected = g_comm && g_comm->reconnectWithPort("\\\\.\\" + port.toStdString());
        QJsonObject r = ok(connected);
        if (!connected) r.insert("msg", QString("No se pudo abrir %1").arg(port));
        return r;
    }

    return QJsonObject{ {"ok", false}, {"msg", "Orden desconocida: " + cmd} };
}

void WebServer::updateGaitFromKeys()
{
    // Idéntico a MainWindow::updateTrotVelocity
    float vx = 0.0f, vy = 0.0f;
    if (keyUp_)    vx += 0.04f;  // Adelante +X
    if (keyDown_)  vx -= 0.04f;  // Atrás -X
    if (keyLeft_)  vy -= 0.04f;  // Izquierda -Y (Y es positivo a la derecha)
    if (keyRight_) vy += 0.04f;  // Derecha +Y

    const bool walking = std::abs(vx) > 0.001f || std::abs(vy) > 0.001f;
    if (walking) {
        if (!robot_->isGaitActive()) {
            // Sólo se arranca la marcha desde la pose de reposo erguida: si no lo está,
            // se hace un reset de pose (levantarse) y se arranca en cuanto llegue.
            if (!isRobotInRestPose()) {
                if (!pendingGaitStart_) {
                    pendingGaitStart_ = true;
                    pendingGaitRetries_ = 0;
                    resetPose();
                }
                return;
            }
            robot_->startGait();
        }
        robot_->setGaitVelocity(vx, vy);
    } else {
        pendingGaitStart_ = false;
        pendingGaitRetries_ = 0;
        if (robot_->isGaitActive()) robot_->stopGait();
    }
}

void WebServer::resetPose()
{
    std::fill(body_, body_ + 6, 0);
    legIkNominal_ = true;
    // Las pestañas ponen a cero los sliders del cuerpo y en nominal los de la pata
    broadcastJson(QJsonObject{ {"type", "poseReset"}, {"body", bodyPoseJson()} });
    robot_->standUp();
}

bool WebServer::isRobotInRestPose() const
{
    for (int v : body_) if (v != 0) return false;   // 1. cuerpo en posición neutra
    if (!legIkNominal_) return false;               // 2. IK de pata en nominal
    return robot_ && robot_->isAtRestPose();        // 3. robot erguido y en reposo físico
}

void WebServer::checkPendingGaitStart()
{
    if (!pendingGaitStart_ || robot_->isSequenceActive()) return;
    if (isRobotInRestPose()) {
        pendingGaitStart_ = false;
        pendingGaitRetries_ = 0;
        if (keyUp_ || keyDown_ || keyLeft_ || keyRight_) updateGaitFromKeys();
    } else if (++pendingGaitRetries_ > 5) {
        pendingGaitStart_ = false;
        pendingGaitRetries_ = 0;
    }
}

void WebServer::broadcastJson(const QJsonObject& obj)
{
    for (auto it = clients_.cbegin(); it != clients_.cend(); ++it)
        if (it->isWebSocket) sendJson(it.key(), obj);
}

// ============================================================================
// Estado
// ============================================================================

QJsonObject WebServer::bodyPoseJson() const
{
    return QJsonObject{
        {"x", body_[0]}, {"y", body_[1]}, {"z", body_[2]},
        {"roll", body_[3]}, {"pitch", body_[4]}, {"yaw", body_[5]},
    };
}

QJsonObject WebServer::buildState() const
{
    auto r1 = [](float v) { return std::round(v * 10.0f) / 10.0f; };

    QJsonArray motors;
    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl) continue;
        const bool online = ctrl->motor_online.load(std::memory_order_relaxed);
        const bool active = ctrl->active.load(std::memory_order_relaxed);
        const float real  = ctrl->last_known_pos.load(std::memory_order_relaxed);
        // Ángulo comandado = el que realmente se envía al motor en cada tick
        // (consigna de la trayectoria trapezoidal, o la directa). Si el motor
        // está apagado no hay consigna, así que mostramos la posición real.
        float cmd = real;
        if (active) {
            cmd = ctrl->is_trap_traj.load(std::memory_order_relaxed)
                ? ctrl->current_traj_pos.load(std::memory_order_relaxed)
                : ctrl->target_pos.load(std::memory_order_relaxed);
        }
        motors.append(QJsonObject{
            {"id", int(id)},
            {"online", online},
            {"active", active},
            {"pos", r1(real * RAD_TO_DEG)},
            {"cmd", r1(cmd * RAD_TO_DEG)},
            {"kp", ctrl->target_kp.load(std::memory_order_relaxed)},
        });
    }

    QJsonArray grounded;
    for (int leg = 1; leg <= 4; ++leg) grounded.append(robot_->getLeg(leg).isGrounded());

    return QJsonObject{
        {"type", "state"},
        {"sim", sim_},
        {"usb", g_comm && g_comm->connected()},
        {"can", robot_->isAnyMotorOnline()},
        {"port", currentPort_},
        {"gait", QJsonObject{ {"active", robot_->isGaitActive()}, {"type", gaitType_}, {"pending", pendingGaitStart_} }},
        {"sequence", robot_->isSequenceActive()},
        {"body", bodyPoseJson()},
        {"grounded", grounded},
        {"motors", motors},
    };
}

// ============================================================================
// Modo simulación (--sim)
//
// Los motores que NO están conectados se marcan como activos y el servidor avanza
// su trayectoria trapezoidal a 50 Hz (sólo calcula la consigna, no envía nada).
// Así las órdenes de la interfaz (levantarse, postura, marcha, IK...) mueven el
// gemelo "Comandado" aunque no haya araña. Los motores conectados no se tocan.
// ============================================================================

void WebServer::simActivateOfflineMotors(int onlyId)
{
    for (uint32_t id : MOTOR_IDS) {
        if (onlyId && int(id) != onlyId) continue;
        MotorController* ctrl = g_controllers[id];
        if (!ctrl || ctrl->motor_online.load()) continue;  // los motores reales no se tocan
        if (!ctrl->active.load()) {
            ctrl->current_traj_pos.store(ctrl->target_pos.load());
            ctrl->current_traj_vel.store(0.0f);
            ctrl->last_tick_time_ms.store(0);
            ctrl->applyStiffness(3);
            ctrl->active.store(true);
        }
    }
}

void WebServer::simStepTrajectories()
{
    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl || ctrl->motor_online.load() || !ctrl->active.load()) continue;
        if (ctrl->is_trap_traj.load()) ctrl->step_trajectory(now_ms);  // sólo calcula la consigna, no envía nada
    }
}

void WebServer::broadcastState()
{
    // Igual que el timer de 100 ms de MainWindow::updateUI
    if (++pendingCheckDivider_ >= 3) { pendingCheckDivider_ = 0; checkPendingGaitStart(); }

    bool any = false;
    for (auto it = clients_.cbegin(); it != clients_.cend(); ++it)
        if (it->isWebSocket) { any = true; break; }
    if (!any) return;

    const QByteArray json = QJsonDocument(buildState()).toJson(QJsonDocument::Compact);
    for (auto it = clients_.cbegin(); it != clients_.cend(); ++it) {
        QTcpSocket* sock = it.key();
        // Si un cliente va con retraso, no le acumulamos estados viejos
        if (it->isWebSocket && sock->bytesToWrite() < 256 * 1024) sendWs(sock, json);
    }
}
