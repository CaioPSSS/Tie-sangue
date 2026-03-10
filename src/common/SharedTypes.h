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
    float yaw_deg;
    float earth_z_accel;        // Aceleração inercial para o Variômetro (Filtro Vertical)
    
    // --- DADOS DO CORE 0 PARA O CORE 1 (Navegação/Autonomia) ---
    float desired_roll_cmd;     // Comando gerado pelo L1 Guidance
    float desired_pitch_cmd;    // Comando gerado pelo TECS
    float desired_throttle;     // Comando gerado pelo TECS (0.0 a 1.0)
    float cog_error;          // Variável crítica para o L1 Guidance corrigir a proa em relação à linha de rumo
    
    // --- DADOS DO NAVEGADOR (Core 0/0) ---
    float altitude_m;           // Fundido pelo VerticalFilter
    float vertical_speed_ms;    // Fundido pelo VerticalFilter
    int32_t lat, lon;           // Coordenadas puras do GPS
    float ground_speed_ms;
    float gps_course_deg;       // COG (Course over ground)
    bool gps_fix;

    // --- DADOS DO RÁDIO LORA (C2 - Command & Control) ---
    float rc_roll_cmd;      // Convertido para graus (ex: -45.0 a 45.0)
    float rc_pitch_cmd;     // Convertido para graus (ex: -45.0 a 45.0)
    float rc_throttle_pwm;  // Convertido de volta para 1000 a 2000us para o Mixer
    uint8_t current_mode;
    bool is_armed; 
    uint32_t last_rc_packet_ms; // CRÍTICO PARA O FAILSAFE!
    bool has_received_first_packet; // Para evitar o Failsafe antes do primeiro contato com a base
    bool failsafe_override; //Bloqueio de hierarquia para impedir que o Rádio sobrescreva emergências do sistema
    
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

// --------------------------------------------------------
// 3. ESTRUTURA DE COMANDOS (UPLINK: BASE -> DRONE) - 8 BYTES
// O seu "Controle Remoto" via LoRa
// --------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t sync_header;    // Identificador (Ex: 0xBB)
    int8_t  cmd_roll;       // Stick de Rolagem (-100 a 100)
    int8_t  cmd_pitch;      // Stick de Arfagem (-100 a 100)
    uint8_t cmd_throttle;   // Stick de Acelerador (0 a 100)
    uint8_t cmd_mode;       // Chave de Modo de Voo (Manual, Angle, Auto, RTH)
    uint8_t arm_switch;     // Chave de Armar Motores (0 = Desarmado, 1 = Armado)
    uint8_t checksum_crc8;  // Validador
} PacketUplinkLoRa_t;

// --------------------------------------------------------
// 4. ESTRUTURA DE UPLOAD DE MISSÃO LORA (BASE -> DRONE)
// --------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  sync_header;   // Identificador EXCLUSIVO: 0xCC
    uint8_t  wp_index;      // Qual posição na RAM (0 a 19)
    int32_t  lat_e7;        // Latitude * 10^7
    int32_t  lon_e7;        // Longitude * 10^7
    int16_t  alt_m;         // Altitude em metros
    uint8_t  speed_ms;      // Velocidade alvo em m/s
    uint8_t  checksum_crc8; 
} PacketWaypointLoRa_t;

#endif