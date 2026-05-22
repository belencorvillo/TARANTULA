    #pragma once
#include <string>
#include <cstdint>

//BAUDRATE CAN: 500 KBPS
//BAUDRATE SERIAL (USB): 2000000 BPS

namespace Config {
    // 1. CONFIGURACIÓN DEL PUERTO SERIE (WAVESHARE)
    const std::string SERIAL_PORT = "\\\\.\\COM3"; // Formato seguro para Windows
    const int SERIAL_BAUDRATE = 2000000;
   

    // 2. PROTOCOLO WAVESHARE
    const uint8_t FRAME_HEAD_1 = 0xAA;
    const uint8_t FRAME_HEAD_2 = 0x55;
    const uint8_t CMD_TYPE_DATA = 0x01; //comando para enviar y recibir datos
    const uint8_t CMD_TYPE_CONFIG = 0x02; //comando para configurar

    
    const uint8_t CAN_BAUD_500K = 0x03;
    

    // 3. LÍMITES FÍSICOS DEL MOTOR (FIRMWARE)
    const float P_MIN = -12.5f;
    const float P_MAX = 12.5f;
    const float V_MIN = -65.0f;
    const float V_MAX = 65.0f;
    const float T_MIN = -50.0f;
    const float T_MAX = 50.0f;
    const float KP_MIN = 0.0f;
    const float KP_MAX = 500.0f;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 5.0f;

    // 4. LÍMITES DE SEGURIDAD (SOFTWARE)
    const float SAFE_VEL_LIMIT = 30.0f;
    const float SAFE_CURRENT_LIMIT = 10.0f;
}
