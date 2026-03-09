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
// Esta estrutura transita entre os núcleos usando Mutex.
// --------------------------------------------------------
struct FlightState {
    // --- DADOS DO CORE 1 PARA O CORE 0 ---
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float earth_z_accel;        // Aceleração inercial para o Variômetro (Filtro Vertical)
    
    // --- DADOS DO CORE 0 PARA O CORE 1 (Navegação/Autonomia) ---
    float desired_roll_cmd;     // Comando gerado pelo L1 Guidance
    float desired_pitch_cmd;    // Comando gerado pelo TECS
    float desired_throttle;     // Comando gerado pelo TECS (0.0 a 1.0)
    
    // --- DADOS DO NAVEGADOR (Core 0/0) ---
    float altitude_m;           // Fundido pelo VerticalFilter
    float vertical_speed_ms;    // Fundido pelo VerticalFilter
    int32_t lat, lon;           // Coordenadas puras do GPS
    float ground_speed_ms;
    float gps_course_deg;       // COG (Course over ground)
    bool gps_fix;
    
    // --- DADOS DE SISTEMA ---
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