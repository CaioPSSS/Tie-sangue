#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// Inclusão das nossas classes de engenharia
#include "control/PID.h"
#include "control/Mixer.h"
#include "guidance/AHRS.h"
#include "guidance/VerticalFilter.h"
#include "guidance/L1_Guidance.h"
#include "guidance/TECS.h"
#include "sensors/SensorManager.h"
#include "hardware/OutputManager.h"
#include "common/SharedTypes.h"
#include "comms/LoRaManager.h"

// ==========================================
// VARIÁVEIS GLOBAIS DE IPC (Inter-Process Comm)
// ==========================================
FlightState globalState;
SemaphoreHandle_t stateMutex;          // Protege a escrita/leitura do globalState
QueueHandle_t telemetryQueue;          // Fila para enviar dados para a Task LoRa

// ==========================================
// INSTÂNCIA DOS OBJETOS DE VOO (Globais)
// ==========================================
// PIDs de Rate (Taxa) -> P, I, D, FF, Max_I, Max_Out
PID rollRatePID(0.5, 0.1, 0.02, 0.8, 100.0, 500.0);
PID pitchRatePID(0.6, 0.15, 0.03, 0.9, 100.0, 500.0);

// PIDs de Angle (Atitude) -> Apenas P
PID rollAnglePID(4.0, 0.0, 0.0, 0.0, 0.0, 300.0);
PID pitchAnglePID(5.0, 0.0, 0.0, 0.0, 0.0, 300.0);

// Módulos de Estabilidade e Navegação
AHRS ahrs;
ElevonMixer elevonMixer;
VerticalFilter verticalFilter;
L1Guidance l1Guidance;
TECS tecs;

// ==========================================
// DECLARAÇÃO DAS TAREFAS
// ==========================================
void Task_FlightLoop(void *pvParameters);
void Task_Navigation(void *pvParameters);
void Task_GPS_Parser(void *pvParameters);
void Task_LoRa_Comm(void *pvParameters);
void Task_System_Mon(void *pvParameters);

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
    Serial.begin(115200);
    Serial.println("INICIANDO SISTEMA DE CONTROLE - ASA VOADORA V2.0");

    // Inicialização dos Primitivos de Sincronização do FreeRTOS
    stateMutex = xSemaphoreCreateMutex();
    telemetryQueue = xQueueCreate(5, sizeof(PacketTelemetryLoRa_t));

    if (stateMutex == NULL || telemetryQueue == NULL) {
        Serial.println("ERRO CRÍTICO: Falha ao alocar memória para o RTOS.");
        while(1); // Trava o sistema
    }

    //INICIAR O LORA:
    LoRaManager::init();

    //INICIAR OS SENSORES:
    SensorManager::initSensors(); 

    //INICIAR OS ATUADORES:
    OutputManager::init();

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 1 (Prioridade Máxima) ---
    xTaskCreatePinnedToCore(Task_FlightLoop, "FlightLoop", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(Task_Navigation, "NavLoop", 4096, NULL, 4, NULL, 1);

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 0 (Comunicação/IO) ---
    xTaskCreatePinnedToCore(Task_GPS_Parser, "GPSParser", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(Task_LoRa_Comm, "LoRaComm", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(Task_System_Mon, "SysMon", 2048, NULL, 1, NULL, 0);

    vTaskDelete(NULL); // Deleta o void loop()
}

void loop() {}

// ==========================================
// CORE 1: MALHA DE VOO (250Hz) - SOBREVIVÊNCIA
// ==========================================
void Task_FlightLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); 
    const float dt = 0.004f; 

    for(;;) {
        // 1. Ler Sensores (MPU6050)
        // RawIMU imuData;
        // SensorManager::readIMU(imuData);

        // 2. Atualizar AHRS (Filtro Mahony)
        // ahrs.update(imuData.gx, imuData.gy, imuData.gz, imuData.ax, imuData.ay, imuData.az, dt);

        // --- CALCULO TPA (Throttle PID Attenuation) ---
        float current_throttle_pwm = 1500.0f; // TODO: Ler do receptor de rádio (ELRS)
        float throttle_percent = constrain((current_throttle_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
        float tpa_factor = 1.0f;
        if (throttle_percent > 0.5f) {
            tpa_factor = 1.0f - ((throttle_percent - 0.5f) * 0.6f); 
        }

        // --- 3. BUSCAR COMANDOS DO CORE 0 (Rádio LoRa ou Navegação) ---
        float target_roll = 0.0f;
        float target_pitch = 0.0f;
        float current_throttle_pwm = 1000.0f; 
        uint8_t current_mode = MODE_MANUAL;
        
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) { 
            // Lemos os comandos dos Sticks do LoRa
            target_roll = globalState.rc_roll_cmd;
            target_pitch = globalState.rc_pitch_cmd;
            current_throttle_pwm = globalState.rc_throttle_pwm;
            current_mode = globalState.current_mode;
            
            // (Mais tarde, se current_mode == MODE_AUTO, nós substituímos 
            // target_roll pelo globalState.desired_roll_cmd gerado pelo L1 Guidance!)
            
            // Grava os dados da IMU para o Core 0 usar depois
            globalState.roll_deg = ahrs.roll;
            globalState.pitch_deg = ahrs.pitch;
            globalState.earth_z_accel = ahrs.earth_z_accel;
            xSemaphoreGive(stateMutex);
        }

        // --- CÁLCULO DO TPA (Throttle PID Attenuation) ---
        // Agora usando o acelerador real do rádio LoRa!
        float throttle_percent = constrain((current_throttle_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
        float tpa_factor = 1.0f;
        if (throttle_percent > 0.5f) {
            tpa_factor = 1.0f - ((throttle_percent - 0.5f) * 0.6f); 
        }

        // 4. MALHAS DE CONTROLE PID (Angle -> Rate)
        // O avião agora tenta seguir o ângulo que você comandou no joystick do PC!
        float desired_roll_rate = rollAnglePID.compute(target_roll, ahrs.roll, dt, 1.0f);
        float desired_pitch_rate = pitchAnglePID.compute(target_pitch, ahrs.pitch, dt, 1.0f);

        // Aqui aplicamos o tpa_factor apenas na malha de Rate (que sofre com a vibração em alta velocidade)
        float mixer_roll_cmd = rollRatePID.compute(desired_roll_rate, /*imuData.gx*/ 0.0f, dt, tpa_factor);
        float mixer_pitch_cmd = pitchRatePID.compute(desired_pitch_rate, /*imuData.gy*/ 0.0f, dt, tpa_factor);

        // 5. MATRIZ DE ELEVONS
        elevonMixer.compute(mixer_pitch_cmd, mixer_roll_cmd);

        // 6. SAÍDA FÍSICA PARA O HARDWARE
        // Pega os sinais gerados pela matriz e aplica nos pinos dos servos
        OutputManager::writeServos(elevonMixer.servo_left_pwm, elevonMixer.servo_right_pwm);

        // Manda o motor rodar com base no que você pediu no rádio LoRa,
        // mas só permite se a variável is_armed estiver verde (Chave acionada no PC).
        OutputManager::writeMotor(current_throttle_pwm, globalState.is_armed);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 1: MALHA DE NAVEGAÇÃO (50Hz) - INTELIGÊNCIA
// ==========================================
void Task_Navigation(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); 
    const float dt = 0.02f;

    for(;;) {
        // 1. Ler Barômetro e converter pressão para altitude
        // float baro_pressure, baro_temp;
        // SensorManager::readBaro(baro_pressure, baro_temp);
        float baro_alt = 0.0f; // TODO: Implementar fórmula barométrica

        // 2. Variáveis de Estado Local
        float current_earth_z_accel = 0.0f;
        float current_ground_speed = 15.0f; // Mockup (15 m/s)
        float current_lat = 0.0f, current_lon = 0.0f;
        float current_course = 0.0f;

        // Buscar estado seguro gerado pelo Core 1 / GPS
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            current_earth_z_accel = globalState.earth_z_accel;
            current_ground_speed = globalState.ground_speed_ms;
            // current_lat = globalState.lat ...
            xSemaphoreGive(stateMutex);
        }

        // 3. FUSÃO VERTICAL (Baro + AccelZ)
        verticalFilter.update(baro_alt, current_earth_z_accel, dt);

        // 4. ALVOS DE NAVEGAÇÃO (Waypoints)
        // TODO: Estas variáveis virão da Máquina de Estados de Missão
        float target_altitude = 50.0f; 
        float target_speed = 15.0f;
        float wp_prev_lat = 0.0f, wp_prev_lon = 0.0f;
        float wp_next_lat = 0.01f, wp_next_lon = 0.01f;

        // 5. PROCESSAMENTO L1 E TECS
        l1Guidance.compute(current_lat, current_lon, current_ground_speed, current_course, 
                           wp_prev_lat, wp_prev_lon, wp_next_lat, wp_next_lon);

        tecs.compute(target_altitude, verticalFilter.estimated_altitude_m, 
                     verticalFilter.vertical_speed_ms, 
                     target_speed, current_ground_speed, dt);

        // 6. ENVIAR COMANDOS GERADOS PARA O CORE 1
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            globalState.desired_roll_cmd = l1Guidance.roll_cmd_deg;
            globalState.desired_pitch_cmd = tecs.pitch_cmd_deg;
            globalState.desired_throttle = tecs.throttle_cmd_percent;
            
            // Salva altitude filtrada para a Telemetria (LoRa)
            globalState.altitude_m = verticalFilter.estimated_altitude_m; 
            xSemaphoreGive(stateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 0: TAREFAS DE COMUNICAÇÃO LENTA
// ==========================================
void Task_GPS_Parser(void *pvParameters) {
    for(;;) {
        // TODO: Ler Serial2, rodar TinyGPS++, atualizar globalState.lat / lon / course
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void Task_LoRa_Comm(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz (100ms)

    PacketTelemetryLoRa_t telemetryPacket;
    telemetryPacket.sync_header = 0xAA; // Byte fixo de Telemetria (TX)

    PacketUplinkLoRa_t uplinkPacket;
    int8_t last_rssi = -127; // Sinal péssimo por padrão

    // Coloca o módulo em modo de escuta assim que a tarefa inicia
    LoRa.receive(); 

    for(;;) {
        
        // =======================================================
        // 1. OUVIR A BASE (UPLINK - RECEÇÃO RC)
        // =======================================================
        if (LoRaManager::receiveUplink(uplinkPacket, last_rssi)) {
            
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                // Conversão do Stick (-100 a 100) para Ângulo Desejado (-45º a +45º)
                // Se você colocar o stick todo para a direita (100), pede 45 graus de Roll
                globalState.rc_roll_cmd = (uplinkPacket.cmd_roll / 100.0f) * 45.0f;
                globalState.rc_pitch_cmd = (uplinkPacket.cmd_pitch / 100.0f) * 45.0f;
                
                // Conversão do Acelerador (0 a 100) para PWM padrão (1000 a 2000us)
                globalState.rc_throttle_pwm = 1000.0f + (uplinkPacket.cmd_throttle * 10.0f);
                
                globalState.current_mode = uplinkPacket.cmd_mode;
                globalState.is_armed = (uplinkPacket.arm_switch == 1);
                
                // Atualiza o relógio do Failsafe (Reset ao "Deadman Switch")
                globalState.last_rc_packet_ms = millis();
                
                xSemaphoreGive(stateMutex);
            }
        }

        // =======================================================
        // 2. FALAR COM A BASE (TELEMETRIA - TRANSMISSÃO)
        // =======================================================
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            telemetryPacket.fsm_state       = MODE_AUTO; 
            telemetryPacket.altitude_cm     = (int16_t)(globalState.altitude_m * 100.0f);
            telemetryPacket.heading_deg_10  = (uint16_t)(globalState.gps_course_deg * 10.0f);
            telemetryPacket.latitude_gps    = globalState.lat; 
            telemetryPacket.longitude_gps   = globalState.lon; 
            telemetryPacket.battery_volt_mv = (uint16_t)(globalState.battery_voltage * 1000.0f);
            
            // Enviamos para a Ground Station a qualidade do sinal que chegou no avião!
            // Assim o piloto no chão sabe se o avião está a ouvi-lo bem.
            telemetryPacket.rssi_uplink     = last_rssi; 
            
            xSemaphoreGive(stateMutex);
        }

        LoRaManager::sendTelemetry(telemetryPacket); // Envia (Demora uns ~20ms em SF8)

        // =======================================================
        // 3. VOLTAR A ESCUTAR
        // =======================================================
        // Como o sendTelemetry() muda o chip para modo TX, 
        // TEMOS obrigatoriamente de mandá-lo voltar para RX (Escuta).
        LoRa.receive();

        // 4. Aguarda até fechar exatos 100ms desde o início do loop
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_System_Mon(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 Hz

    for(;;) {
        // TODO: Monitorar ADC da Bateria e disparar Failsafe se o rádio cair
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}