#include <QApplication>
#include <QDesktopServices>
#include <QUrl>
#include <QDir>
#include <QFileInfo>
#include <iostream>
#include <memory>
#include "WaveshareInterface.h"
#include "MotorController.h"
#include "Tarantula.h"
#include "MainWindow.h"
#include "WebServer.h"
#include "Config.h"

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

// Busca la carpeta web/ junto al ejecutable o en las carpetas superiores
// (el .exe suele estar en build/ o build/Debug/, y web/ en la raíz del proyecto).
static QString findWebRoot()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 5; ++i) {
        if (QFileInfo(dir.filePath("web/index.html")).isFile())
            return dir.filePath("web");
        if (!dir.cdUp()) break;
    }
    if (QFileInfo(QDir::current().filePath("web/index.html")).isFile())
        return QDir::current().filePath("web");
    return {};
}

const float GEAR_RATIO = 8.0f;
const double SEND_FREQUENCY = 0.02; // 20 ms (50 Hz). La cola asíncrona despacha las 12 tramas en 18 ms (pacing de 1.5 ms necesario para evitar latigazos), dejando 2 ms libres por ciclo.

int main(int argc, char* argv[])
{
#ifdef _WIN32
    timeBeginPeriod(1); // Ajustar la resolución del temporizador de Windows a 1 ms
#endif

    QApplication app(argc, argv);

    std::cout << "Iniciando Waveshare USB-CAN adapter...\n";
    WaveshareInterface comm(Config::SERIAL_PORT, Config::SERIAL_BAUDRATE);
    if (!comm.connect())
        std::cerr << "Error: No se pudo conectar al puerto serial. Modo OFFLINE.\n";

    g_comm = &comm;  //en motorcontroler

    Tarantula robot(comm);
    robot.start();

    // --- Interfaz ---------------------------------------------------------
    //   (por defecto)   interfaz web en el navegador -> http://localhost:8080
    //   --qt            interfaz clásica de Qt (MainWindow)
    //   --port N        puerto del servidor web (por defecto 8080)
    //   --no-browser    no abrir el navegador automáticamente
    //   --sim           modo simulación: los motores desconectados se simulan para
    //                   ver moverse el gemelo "Comandado" sin la araña conectada
    const QStringList args = app.arguments();
    const bool useQt = args.contains("--qt");

    std::unique_ptr<MainWindow> window;
    std::unique_ptr<WebServer> web;

    if (useQt) {
        window = std::make_unique<MainWindow>(&robot);
        window->show();
    } else {
        quint16 port = 8080;
        const int pi = args.indexOf("--port");
        if (pi >= 0 && pi + 1 < args.size()) port = static_cast<quint16>(args[pi + 1].toUInt());

        const QString webRoot = findWebRoot();
        if (webRoot.isEmpty()) {
            std::cerr << "Error: no se encuentra la carpeta 'web' (con index.html). "
                         "Usa --qt para la interfaz clásica.\n";
            return 1;
        }
        web = std::make_unique<WebServer>(&robot, webRoot, args.contains("--sim"));
        const quint16 used = web->start(port);
        if (!used) {
            std::cerr << "Error: no se pudo abrir el servidor web en el puerto " << port << ".\n";
            return 1;
        }
        std::cout << "\n  Interfaz web:  " << web->url().toStdString() << "\n"
                  << "  (cierra esta ventana o pulsa Ctrl+C para salir)\n\n";
        if (!args.contains("--no-browser"))
            QDesktopServices::openUrl(QUrl(web->url()));
    }

    int ret = app.exec();
    web.reset();
    window.reset();

    robot.stop();
    comm.close();

#ifdef _WIN32
    timeEndPeriod(1); // Restaurar la resolución original del temporizador de Windows
#endif

    return ret;
}
