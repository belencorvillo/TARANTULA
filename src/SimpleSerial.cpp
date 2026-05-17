#include "SimpleSerial.h"
#include <iostream>

SimpleSerial::SimpleSerial(std::string portName, int baud)
    : port(portName), baudrate(baud), connected(false), hSerial(INVALID_HANDLE_VALUE) {
}

SimpleSerial::~SimpleSerial() {
    close();
}

bool SimpleSerial::connect() {

    //abrimos el puerto para lectura y escritura creando un fichero con la función CreateFileA de windows.h
    hSerial = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) return false;

    //configuramos Device Control Block (DCB)
    //El DCB es una estructura que contiene todos los parámetros necesarios del puerto serie
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) { close(); return false; }

    dcbSerialParams.BaudRate = baudrate; 
    dcbSerialParams.ByteSize = 8; // paquetes de 8 bits
    dcbSerialParams.StopBits = ONESTOPBIT; //1 bit de parada al final
    dcbSerialParams.Parity = NOPARITY; //sin bit de comprobación 

    if (!SetCommState(hSerial, &dcbSerialParams)) { close(); return false; }

    //COMMTIMEOUTS es una estructura de Windows que define el tiempo que se queda congelado el programa esperando a que llegue un dato
    //si pasan 5ms y no llega nada, pasa y sigue ejecutando código
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 5;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 1;

    SetCommTimeouts(hSerial, &timeouts);
    connected = true;
    return true;
}

void SimpleSerial::close() {
    if (connected) {
        CloseHandle(hSerial);
        connected = false;
    }
}

bool SimpleSerial::writeBytes(const std::vector<uint8_t>& data) {
    if (!connected) return false;
    DWORD bytesSend;
    //escribe los datos a través del handler del USB y devuelve el número de datos enviados
    return WriteFile(hSerial, data.data(), data.size(), &bytesSend, 0);
}

int SimpleSerial::readBytes(std::vector<uint8_t>& buffer, int count) {
    if (!connected) return 0;
    //ensancha el buffer para que quepan count bytes
    buffer.resize(count);
    DWORD bytesRead = 0;
    //lee el puerto de windows
    ReadFile(hSerial, buffer.data(), count, &bytesRead, 0);
   //recorta el buffer
    buffer.resize(bytesRead);
    return bytesRead;

    //este recorte sirve para no leer basura
}

int SimpleSerial::available() {
    if (!connected) return 0;
    COMSTAT status;
    DWORD errors;
    ClearCommError(hSerial, &errors, &status);
    return status.cbInQue;
}

void SimpleSerial::flush() {

    //si vemos que la cabecerra no es 0xAA 0x55 entonces tiramos el mensaje, asumiéndolo como no válido
    if (connected) PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
}
