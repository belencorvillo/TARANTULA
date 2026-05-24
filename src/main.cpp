#include <QApplication>
#include <iostream>
#include "WaveshareInterface.h"
#include "MotorController.h"
#include "Tarantula.h"
#include "MainWindow.h"
#include "Config.h"

const float GEAR_RATIO = 8.0f;
const double SEND_FREQUENCY = 0.015; // 15 ms (66 Hz). La cola asíncrona despacha las 12 tramas en 12 ms (pacing obligatorio de 1.0 ms), dejando 3 ms libres por ciclo.

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    std::cout << "Iniciando Waveshare USB-CAN adapter...\n";
    WaveshareInterface comm(Config::SERIAL_PORT, Config::SERIAL_BAUDRATE);
    if (!comm.connect())
        std::cerr << "Error: No se pudo conectar al puerto serial. Modo OFFLINE.\n";

    g_comm = &comm;  

    Tarantula robot(comm);
    robot.start();

    MainWindow window(&robot);
    window.show();

    int ret = app.exec();

    robot.stop();
    comm.close();
    return ret;
}
