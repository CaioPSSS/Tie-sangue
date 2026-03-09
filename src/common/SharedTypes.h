#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <Arduino.h>

// --------------------------------------------------------
// 1. ESTRUTURA DE TELEMETRIA LORA (COMPACTADA - 18 BYTES)
// Usada pela Task_LoRa_Comm para enviar para a Ground Station
// --------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t   sync_header;      // Métrica de prefixação Sincronizadora (Ex: 0xAA)
    uint8_t   fsm_state;        // Modo de voo atual (Manual, Angle, RTH, etc)
    int16_t   altitude_cm;      // Altitude barométrica filtrada (+- 32km)
    uint16_t  heading_deg_10;   // Proa em décimos de grau (Ex: 3599 = 359.9º)
    int32_t   latitude_gps;     // Lat bruta * 10^7
    int32_t   longitude_gps;    // Lon bruta * 10^7
    uint16_t  battery_volt_mv;  // Tensão da Li-ion em milivolts
    int8_t    rssi_uplink;      // Qualidade do sinal LoRa
    uint8_t   checksum_crc8;    // Validador de integridade do pacote
} PacketTelemetryLoRa_t;

// --------------------------------------------------------
// 2. ESTADO GLOBAL DA AERONAVE (IPC)
// Esta estrutura transita entre os núcleos. 
// --------------------------------------------------------
struct FlightState {
    // Atitude (Core 1)
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    
    // Cinemática Vertical (Filtro Kalman 1D - Core 1/0)
    float altitude_m;
    float vertical_speed_ms;
    
    // Navegação e GPS (Core 0)
    int32_t lat, lon;
    float ground_speed_ms;
    float gps_course_deg; // O famoso COG
    bool gps_fix;
    
    // Sensores
    float battery_voltage;
};

// Modos da Máquina de Estados (FSM)
enum FlightMode {
    MODE_MANUAL = 0,
    MODE_ANGLE,
    MODE_HOLD,
    MODE_AUTO,
    MODE_RTH
};

#endif