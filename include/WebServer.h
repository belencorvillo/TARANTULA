#pragma once
// Servidor local para la interfaz web (carpeta web/).
//
//  - HTTP:      sirve los ficheros estáticos de web/ en http://localhost:<puerto>/
//  - WebSocket: ws://localhost:<puerto>/ws  -> órdenes JSON hacia Tarantula y
//               difusión del estado del robot a 30 Hz.
//
// Sólo usa QtCore + QtNetwork (sin el módulo Qt WebSockets). Escucha únicamente
// en 127.0.0.1, así que no es accesible desde otros equipos de la red.
//
// Seguridad: sólo la pestaña conectada más recientemente puede mandar órdenes
// (las demás quedan en solo lectura). Si esa pestaña se cierra o pierde la
// conexión, se suelta el D-pad y se detiene la marcha.

#include <QObject>
#include <QHash>
#include <QJsonObject>
#include <QString>
#include <QByteArray>

class QTcpServer;
class QTcpSocket;
class QTimer;
class Tarantula;

class WebServer : public QObject {
    Q_OBJECT
public:
    // sim = true -> modo simulación (--sim), ver WebServer::simActivateOfflineMotors
    WebServer(Tarantula* robot, const QString& webRoot, bool sim = false, QObject* parent = nullptr);
    ~WebServer() override;

    // Intenta escuchar en `port` (y en los siguientes si está ocupado). Devuelve el puerto usado o 0.
    quint16 start(quint16 port);
    QString url() const;

private slots:
    void onNewConnection();
    void onReadyRead();
    void onDisconnected();
    void broadcastState();

private:
    struct Client {
        QByteArray buffer;
        bool isWebSocket = false;
        QByteArray fragmented;     // mensajes WebSocket fragmentados
        int fragmentedOpcode = 0;
    };

    // HTTP
    void handleHttp(QTcpSocket* sock, Client& c);
    void serveFile(QTcpSocket* sock, const QString& urlPath, bool headOnly);
    void sendHttpError(QTcpSocket* sock, int code, const char* text);

    // WebSocket
    void processWsFrames(QTcpSocket* sock, Client& c);
    void sendWs(QTcpSocket* sock, const QByteArray& payload, int opcode = 0x1);
    void sendJson(QTcpSocket* sock, const QJsonObject& obj);
    void onWsMessage(QTcpSocket* sock, const QByteArray& text);
    void onWsOpen(QTcpSocket* sock);
    void onWsClosed(QTcpSocket* sock);

    // Órdenes
    QJsonObject handleCommand(const QJsonObject& m);
    void updateGaitFromKeys();
    QJsonObject buildState() const;
    QJsonObject bodyPoseJson() const;

    // Arranque de marcha condicionado a la pose de reposo (igual que MainWindow)
    void resetPose();                 // equivalente a MainWindow::on_btnResetPose_clicked
    bool isRobotInRestPose() const;   // equivalente a MainWindow::isRobotInRestPose
    void checkPendingGaitStart();     // equivalente al bloque pending_gait_start_ de MainWindow::updateUI
    void broadcastJson(const QJsonObject& obj);

    // Modo simulación (--sim): motores desconectados simulados para probar sin robot
    bool    sim_ = false;
    QTimer* simTimer_ = nullptr;
    void simActivateOfflineMotors(int onlyId = 0);
    void simStepTrajectories();

    Tarantula*  robot_;
    QString     webRoot_;
    QTcpServer* server_ = nullptr;
    QTimer*     stateTimer_ = nullptr;
    QHash<QTcpSocket*, Client> clients_;
    QTcpSocket* controller_ = nullptr;   // pestaña con permiso de mando

    // Estado de la interfaz (equivalente a los miembros de MainWindow)
    bool keyUp_ = false, keyDown_ = false, keyLeft_ = false, keyRight_ = false;
    int  gaitType_ = 0;
    int  body_[6] = { 0, 0, 0, 0, 0, 0 };  // x, y, z [mm], roll, pitch, yaw [°] — última postura válida
    bool legIkNominal_ = true;             // sliders IK de pata en nominal (495, 0, 0)
    bool pendingGaitStart_ = false;        // marcha pedida, esperando a que el robot vuelva a reposo
    int  pendingGaitRetries_ = 0;
    int  pendingCheckDivider_ = 0;
    QString currentPort_;
};
