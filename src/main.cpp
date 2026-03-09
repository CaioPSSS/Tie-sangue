#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "common/SharedTypes.h"

// Inclusão das nossas classes de engenharia
#include "control/PID.h"
#include "control/Mixer.h"
#include "guidance/AHRS.h"
#include "guidance/VerticalFilter.h"
#include "guidance/L1_Guidance.h"
#include "guidance/TECS.h"
// #include "sensors/SensorManager.h" // Descomente quando criar fisicamente o arquivo

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

    // ONDE INICIAR OS SENSORES:
    // SensorManager::initSensors(); 

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

        // 3. BUSCAR COMANDOS DO CORE 0 (Navegação Automática)
        float target_roll = 0.0f;
        float target_pitch = 0.0f;
        
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) { // Não bloqueia (timeout 0)
            target_roll = globalState.desired_roll_cmd;
            target_pitch = globalState.desired_pitch_cmd;
            
            // Grava os dados da IMU para o Core 0 usar depois
            globalState.roll_deg = ahrs.roll;
            globalState.pitch_deg = ahrs.pitch;
            globalState.earth_z_accel = ahrs.earth_z_accel;
            xSemaphoreGive(stateMutex);
        }

        // 4. MALHAS DE CONTROLE PID (Angle -> Rate)
        float desired_roll_rate = rollAnglePID.compute(target_roll, ahrs.roll, dt, 1.0f);
        float desired_pitch_rate = pitchAnglePID.compute(target_pitch, ahrs.pitch, dt, 1.0f);

        // Aqui aplicamos o tpa_factor apenas na malha de Rate (que sofre com a vibração em alta velocidade)
        float mixer_roll_cmd = rollRatePID.compute(desired_roll_rate, /*imuData.gx*/ 0.0f, dt, tpa_factor);
        float mixer_pitch_cmd = pitchRatePID.compute(desired_pitch_rate, /*imuData.gy*/ 0.0f, dt, tpa_factor);

        // 5. MATRIZ DE ELEVONS E SAÍDA PWM
        elevonMixer.compute(mixer_pitch_cmd, mixer_roll_cmd);
        // servoLeft.writeMicroseconds(elevonMixer.servo_left_pwm);
        // servoRight.writeMicroseconds(elevonMixer.servo_right_pwm);
        
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
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz

    PacketTelemetryLoRa_t telemetryPacket;

    for(;;) {
        // TODO: Ler globalState, preencher telemetryPacket e enviar SPI
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