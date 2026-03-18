#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_task_wdt.h>

// Inclusão das nossas classes de engenharia
#include "control/PID.h"
#include "control/Mixer.h"
#include "guidance/FlightModeManager.h"
#include "guidance/AHRS.h"
#include "guidance/VerticalFilter.h"
#include "guidance/L1_Guidance.h"
#include "guidance/TECS.h"
#include "guidance/MissionManager.h"
#include "sensors/SensorManager.h"
#include "sensors/GPSManager.h"
#include "sensors/BatteryManager.h"
#include "hardware/OutputManager.h"
#include "hardware/LEDManager.h"
#include "common/SharedTypes.h"
#include "comms/LoRaManager.h"


// ==========================================
// VARIÁVEIS GLOBAIS DE IPC (Inter-Process Comm)
// ==========================================
FlightState globalState;
SemaphoreHandle_t stateMutex;          // Protege a escrita/leitura do globalState e MissionManager
QueueHandle_t telemetryQueue;          // Fila para enviar dados para a Task LoRa

// ==========================================
// INSTÂNCIA DOS OBJETOS DE VOO (Globais)
// ==========================================
// PIDs de Rate (Taxa) -> P, I, D, FF, Max_I, Max_Out
PID rollRatePID(0.5, 0.1, 0.02, 0.8, 100.0, 500.0);
PID pitchRatePID(0.4, 0.10, 0.02, 0.6, 100.0, 500.0);

// PIDs de Angle (Atitude) -> Apenas P
PID rollAnglePID(4.0, 0.0, 0.0, 0.0, 0.0, 300.0);
PID pitchAnglePID(3.5, 0.0, 0.0, 0.0, 0.0, 300.0);

// Módulos de Estabilidade e Navegação
AHRS ahrs;
ElevonMixer elevonMixer;
VerticalFilter verticalFilter;
L1Guidance l1Guidance;
TECS tecs;
FlightModeManager fsm;
MissionManager missionManager;

// ==========================================
// DECLARAÇÃO DAS TAREFAS
// ==========================================
void Task_FlightLoop(void *pvParameters);
void Task_Navigation(void *pvParameters);
void Task_GPS_Parser(void *pvParameters);
void Task_LoRa_Comm(void *pvParameters);
void Task_System_Mon(void *pvParameters);
void Task_Lights(void *pvParameters);

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
    Serial.begin(115200);
    Serial.println("INICIANDO SISTEMA DE CONTROLE - ASA VOADORA V2.0 (GOLD EDITION)");

    // Inicialização dos Primitivos de Sincronização do FreeRTOS
    stateMutex = xSemaphoreCreateMutex();
    telemetryQueue = xQueueCreate(5, sizeof(PacketTelemetryLoRa_t));
    
    if (stateMutex == NULL || telemetryQueue == NULL) {
        Serial.println("ERRO CRÍTICO: Falha ao alocar memória para o RTOS.");
        while(1); // Trava o sistema
    }

    // Inicializa o WDT global com timeout de 2 segundos (O dobro do loop mais lento)
    esp_task_wdt_init(2, true);

    // ==========================================
    // INICIALIZAÇÃO DOS MÓDULOS DE HARDWARE
    // ==========================================
    LoRaManager::init();
    SensorManager::initSensors();
    
    // FASE 1: CALIBRAÇÕES CRÍTICAS DE SOLO (Giroscópio e Ground Zero AGL)
    SensorManager::calibrateIMU();
    SensorManager::calibrateBaro();
    
    OutputManager::init();
    GPSManager::init();
    BatteryManager::init();
    LEDManager::init();

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 1 (Prioridade Máxima) ---
    xTaskCreatePinnedToCore(Task_FlightLoop, "FlightLoop", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(Task_Navigation, "NavLoop", 4096, NULL, 4, NULL, 1);

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 0 (Comunicação/IO) ---
    xTaskCreatePinnedToCore(Task_GPS_Parser, "GPSParser", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(Task_LoRa_Comm, "LoRaComm", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(Task_System_Mon, "SysMon", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_Lights, "Lights", 2048, NULL, 1, NULL, 0);

    vTaskDelete(NULL); // Deleta o void loop() para libertar memória
}

void loop() {}

// ==========================================
// CORE 1: MALHA DE VOO (250Hz) - SOBREVIVÊNCIA
// ==========================================
void Task_FlightLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); 
    
    uint32_t last_time_us = micros(); // Inicializa o relógio

    esp_task_wdt_add(NULL); // Adiciona a Task de Voo ao WDT para monitoramento de travamentos

    for(;;) {
        uint32_t now_us = micros();
        float dt = (now_us - last_time_us) / 1000000.0f; // Cálculo dinâmico cravado
        last_time_us = now_us;
        
        // Proteção extra: se o dt explodir por um travamento, limita a 10ms
        if (dt > 0.010f) dt = 0.004f;

        esp_task_wdt_reset(); // Reseta o WDT a cada ciclo para evitar resets indesejados
        
        // 1. Ler Sensores Rápidos (MPU6050)
        RawIMU imuData;
        SensorManager::readIMU(imuData);

        // ========================================================
        // 2. BUSCAR COMANDOS DO CORE 0 (Rádio Humano + Navegação)
        // ========================================================
        float rc_roll = 0.0f, rc_pitch = 0.0f, rc_throttle = 1000.0f;
        float nav_roll = 0.0f, nav_pitch = 0.0f, nav_throttle = 0.0f;
        uint8_t current_mode = MODE_MANUAL;
        bool is_armed = false;
        
        // Variáveis de isolamento de Mutex para segurança de memória
        float ground_speed_local = 0.0f;
        float cog_error_local = 0.0f;
        
        // Timeout 0: Se o Core 0 estiver ocupado, não trava o Core 1. Usa os valores do ciclo passado.
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) { 
            rc_roll = globalState.rc_roll_cmd;
            rc_pitch = globalState.rc_pitch_cmd;
            rc_throttle = globalState.rc_throttle_pwm;
            current_mode = globalState.current_mode;
            is_armed = globalState.is_armed;
            
            nav_roll = globalState.desired_roll_cmd;
            nav_pitch = globalState.desired_pitch_cmd;
            nav_throttle = globalState.desired_throttle;

            // Extrai as variáveis críticas de forma segura (sem Fuga de Mutex)
            ground_speed_local = globalState.ground_speed_ms;
            cog_error_local = globalState.cog_error;
            
            // Grava os dados limpos da IMU para o Core 0 usar
            globalState.roll_deg = ahrs.roll;
            globalState.pitch_deg = ahrs.pitch;
            globalState.yaw_deg = ahrs.yaw; // Essencial para o cálculo de desvio no Core 0
            globalState.earth_z_accel = ahrs.earth_z_accel;
            
            xSemaphoreGive(stateMutex);
        }

        // ========================================================
        // 3. FUSÃO SENSORIAL (AHRS)
        // ========================================================
        // O AHRS é atualizado *agora*, recebendo a injeção do cog_error_local para anular o drift
        ahrs.update(imuData.gx, imuData.gy, imuData.gz, imuData.ax, imuData.ay, imuData.az, dt, cog_error_local);

        // ========================================================
        // 4. MÁQUINA DE ESTADOS (O Cérebro da Asa)
        // ========================================================
        fsm.update(current_mode, 
                   rc_roll, rc_pitch, rc_throttle,
                   nav_roll, nav_pitch, nav_throttle,
                   dt,
                   rollAnglePID, pitchAnglePID, rollRatePID, pitchRatePID);

        // ========================================================
        // 5. MALHAS DE CONTROLE PID E MIXER (Músculos)
        // ========================================================
        float mixer_roll_cmd = 0.0f;
        float mixer_pitch_cmd = 0.0f;

        if (fsm.bypass_pids) {
            mixer_roll_cmd = fsm.final_roll_target;
            mixer_pitch_cmd = fsm.final_pitch_target;
        } else {
            // Cálculo dinâmico do TPA de forma totalmente Thread-Safe
            float tpa_factor = 1.0f;
            
            if (current_mode == MODE_AUTO || current_mode == MODE_RTH || current_mode == MODE_HOLD) {
                // TPA baseado na Cinética Real (GPS Speed Local) para modos autónomos
                if (ground_speed_local > 15.0f) {
                    tpa_factor = 1.0f - ((ground_speed_local - 15.0f) * 0.03f);
                }
            } else {
                // TPA Clássico baseado no acelerador para modos manuais
                float throttle_percent = constrain((fsm.final_throttle_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
                if (throttle_percent > 0.5f) {
                    tpa_factor = 1.0f - ((throttle_percent - 0.5f) * 0.6f); 
                }
            }
            
            // Saturação de proteção do TPA
            tpa_factor = constrain(tpa_factor, 0.4f, 1.0f);

            // Cascata PID (Angle -> Rate)
            float desired_roll_rate = rollAnglePID.compute(fsm.final_roll_target, ahrs.roll, dt, 1.0f);
            float desired_pitch_rate = pitchAnglePID.compute(fsm.final_pitch_target, ahrs.pitch, dt, 1.0f);

            mixer_roll_cmd = rollRatePID.compute(desired_roll_rate, imuData.gx, dt, tpa_factor);
            mixer_pitch_cmd = pitchRatePID.compute(desired_pitch_rate, imuData.gy, dt, tpa_factor);
        }

        // ========================================================
        // 6. MATRIZ DE ELEVONS E SAÍDA DE POTÊNCIA
        // ========================================================
        elevonMixer.compute(mixer_pitch_cmd, mixer_roll_cmd);
        OutputManager::writeServos(elevonMixer.servo_left_pwm, elevonMixer.servo_right_pwm);
        OutputManager::writeMotor(fsm.final_throttle_pwm, is_armed);
        
        // Cede processamento mantendo o loop em rigorosos 250Hz
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 0: MALHA DE NAVEGAÇÃO (50Hz)
// ==========================================
void Task_Navigation(void *pvParameters) {
    esp_task_wdt_add(NULL); // [CORREÇÃO] Adiciona a Task de Navegação ao WDT!
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); 
    
    uint32_t last_time_us = micros(); 

    for(;;) {
        esp_task_wdt_reset(); // [CORREÇÃO] Reseta o WDT a cada ciclo

        uint32_t now_us = micros();
        float dt = (now_us - last_time_us) / 1000000.0f; 
        last_time_us = now_us;
        
        // Proteção extra: a navegação corre a 50Hz (20ms). Limita a 50ms máximo se houver atraso. E a 0.5ms mínimo para evitar divisão por zero.
        if (dt > 0.050f) dt = 0.020f;
        if (dt <= 0.001f) dt = 0.020f;

        // 1. Variáveis de Estado Local
        float current_earth_z_accel = 0.0f;
        float current_ground_speed = 0.0f; 
        
        // [CORREÇÃO SPRINT 1] DOUBLE (64-bits) para evitar truncamento submétrico de GPS!
        double current_lat = 0.0;
        double current_lon = 0.0;
        
        float current_course = 0.0f;
        float ahrs_yaw = 0.0f; // [CORREÇÃO SPRINT 1] Variável para ler o Yaw do Core 1 em segurança
        
        bool  has_gps_fix = false;
        bool  is_armed_now = false;
        uint8_t current_mode_now = MODE_MANUAL;
        uint32_t last_rc_packet = 0;
        bool has_first_packet = false; // [CORREÇÃO SPRINT 2]

        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            current_earth_z_accel = globalState.earth_z_accel;
            current_ground_speed = globalState.ground_speed_ms;
            
            // Re-conversão para decimais puros com cast explícito para double
            current_lat = (double)globalState.lat / 10000000.0;
            current_lon = (double)globalState.lon / 10000000.0;
            current_course = globalState.gps_course_deg;
            
            // Lendo as novas variáveis
            ahrs_yaw = globalState.yaw_deg; 
            has_first_packet = globalState.has_received_first_packet;
            
            has_gps_fix = globalState.gps_fix;
            is_armed_now = globalState.is_armed;
            current_mode_now = globalState.current_mode;
            last_rc_packet = globalState.last_rc_packet_ms;
            
            xSemaphoreGive(stateMutex);
        }

        // 2. FUSÃO VERTICAL (Variómetro / Kalman 1D)
        float baro_pressure, baro_temp;
        if (SensorManager::readBaro(baro_pressure, baro_temp)) {
            float baro_alt = SensorManager::getAltitudeAGL(baro_pressure);
            verticalFilter.update(baro_alt, current_earth_z_accel, dt);
        } else {
            verticalFilter.update(verticalFilter.estimated_altitude_m, current_earth_z_accel, dt);
        }

        // 3. LOCK DO HOME POINT
        static bool home_locked = false;
        if (!home_locked && has_gps_fix && is_armed_now) {
            if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                missionManager.setHome(current_lat, current_lon, verticalFilter.estimated_altitude_m);
                xSemaphoreGive(stateMutex);
            }
            home_locked = true;
            Serial.println(">>> HOME POINT TRAVADO COM SUCESSO! <<<");
        }

        // 4. ATUALIZA A MÁQUINA DE MISSÃO E EXTRAI ALVOS
        // [CORREÇÃO SPRINT 2] Failsafe Fantasma resolvido: Só dispara se já tiver apanhado sinal alguma vez
        bool is_failsafe_active = has_first_packet && ((millis() - last_rc_packet) > 1500);
        bool force_rth = (current_mode_now == MODE_RTH) || is_failsafe_active;

        double wp_prev_lat, wp_prev_lon, wp_next_lat, wp_next_lon; // DOUBLE!
        float target_alt = 0.0f, target_speed = 0.0f;

        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            missionManager.update((float)current_lat, (float)current_lon, verticalFilter.estimated_altitude_m, force_rth);
            
            wp_prev_lat   = missionManager.getPrevLat();
            wp_prev_lon   = missionManager.getPrevLon();
            wp_next_lat   = missionManager.getActiveLat();
            wp_next_lon   = missionManager.getActiveLon();
            target_alt    = missionManager.getActiveAltitude();
            target_speed  = missionManager.getActiveSpeed();
            xSemaphoreGive(stateMutex);
        }

        // 5. PROCESSAMENTO L1 E TECS
        
        // [CORREÇÃO SPRINT 1] Cálculo do COG Error (Mas SEM aceder diretamente ao objeto AHRS!)
        float cog_error_calc = 0.0f;
        if (has_gps_fix && current_ground_speed > 3.0f) {
            cog_error_calc = current_course - ahrs_yaw;
            while (cog_error_calc > 180.0f) cog_error_calc -= 360.0f;
            while (cog_error_calc < -180.0f) cog_error_calc += 360.0f;
        }

        if (has_gps_fix) {
            l1Guidance.compute(current_lat, current_lon, current_ground_speed, current_course, 
                               wp_prev_lat, wp_prev_lon, wp_next_lat, wp_next_lon);
        } else {
            l1Guidance.roll_cmd_deg = 0.0f;
        }

       tecs.compute(target_alt, verticalFilter.estimated_altitude_m, 
                     verticalFilter.vertical_speed_ms, 
                     target_speed, current_ground_speed, dt);

        // 6. ENVIAR COMANDOS PARA O CORE 1
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            globalState.desired_roll_cmd = l1Guidance.roll_cmd_deg;
            globalState.desired_pitch_cmd = tecs.pitch_cmd_deg;
            globalState.desired_throttle = tecs.throttle_cmd_percent;
            
            globalState.altitude_m = verticalFilter.estimated_altitude_m; 
            globalState.vertical_speed_ms = verticalFilter.vertical_speed_ms;
            
            // [CORREÇÃO SPRINT 1] Envia o erro pro Core 1 processar suavemente na IMU
            globalState.cog_error = cog_error_calc; 
            
            xSemaphoreGive(stateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 0: TAREFAS DE COMUNICAÇÃO E SISTEMA
// ==========================================
void Task_GPS_Parser(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for(;;) {
        esp_task_wdt_reset();
        if (GPSManager::update()) {
            if (GPSManager::hasValidFix()) {
                if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                    globalState.lat = GPSManager::getLatitudeE7();
                    globalState.lon = GPSManager::getLongitudeE7();
                    globalState.ground_speed_ms = GPSManager::getSpeedMPS();
                    globalState.gps_course_deg = GPSManager::getCourseDeg();
                    globalState.gps_fix = true;
                    xSemaphoreGive(stateMutex);
                }
            } else {
                if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                    globalState.gps_fix = false;
                    xSemaphoreGive(stateMutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void Task_LoRa_Comm(void *pvParameters) {
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz

    PacketTelemetryLoRa_t telemetryPacket;
    telemetryPacket.sync_header = 0xAA; 

    PacketUplinkLoRa_t uplinkPacket;
    PacketWaypointLoRa_t wpPacket;
    int8_t last_rssi = -127; 

    LoRa.receive(); 

    for(;;) {
        esp_task_wdt_reset();
        // 1. OUVIR A BASE (MULTIPLEXAÇÃO)
        LoRaPacketType pktType = LoRaManager::receive(uplinkPacket, wpPacket, last_rssi);

        // Leitura da bateria em rotina de 10 Hz
        float current_vbat = BatteryManager::readVoltage();

        if (pktType == PACKET_RC_UPLINK) {
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                globalState.has_received_first_packet = true;
                globalState.last_rc_packet_ms = millis();
                globalState.rc_roll_cmd = (uplinkPacket.cmd_roll / 100.0f) * 45.0f;
                globalState.rc_pitch_cmd = (uplinkPacket.cmd_pitch / 100.0f) * 45.0f;
                globalState.rc_throttle_pwm = 1000.0f + (uplinkPacket.cmd_throttle * 10.0f);
                globalState.current_mode = uplinkPacket.cmd_mode;
                globalState.is_armed = (uplinkPacket.arm_switch == 1);
                globalState.battery_voltage = current_vbat;

                if (!globalState.failsafe_override) {
                    uint8_t req_mode = uplinkPacket.cmd_mode;
                    
                    // Impede ativação de modos autónomos se não houver fixação GPS!
                    if ((req_mode == MODE_AUTO || req_mode == MODE_RTH || req_mode == MODE_HOLD) && !globalState.gps_fix) {
                        globalState.current_mode = MODE_ANGLE; // Degradação segura
                    } else {
                        globalState.current_mode = req_mode;
                    }
                }

                xSemaphoreGive(stateMutex);
            }
        } 
        else if (pktType == PACKET_WAYPOINT) {
            float lat_f = wpPacket.lat_e7 / 10000000.0f;
            float lon_f = wpPacket.lon_e7 / 10000000.0f;

            // Protege o MissionManager com Mutex contra conflitos com a Task_Navigation
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                missionManager.saveWaypoint(wpPacket.wp_index, (double)lat_f, (double)lon_f, wpPacket.alt_m, wpPacket.speed_ms);
                xSemaphoreGive(stateMutex);
            }
        }

        // 2. FALAR COM A BASE (TELEMETRIA)
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            telemetryPacket.fsm_state       = globalState.current_mode; 
            telemetryPacket.altitude_cm     = (int16_t)(globalState.altitude_m * 100.0f);
            
            // --- CONVERSÃO DA INCLINAÇÃO PARA A ESTAÇÃO DE SOLO ---
            // Multiplicamos por 10 para não perder a casa decimal num número inteiro
            telemetryPacket.roll_deg_10     = (int16_t)(globalState.roll_deg * 10.0f);
            telemetryPacket.pitch_deg_10    = (int16_t)(globalState.pitch_deg * 10.0f);
            // ------------------------------------------------------
            
            telemetryPacket.heading_deg_10  = (uint16_t)(globalState.gps_course_deg * 10.0f);
            telemetryPacket.latitude_gps    = globalState.lat; 
            telemetryPacket.longitude_gps   = globalState.lon; 
            telemetryPacket.battery_volt_mv = (uint16_t)(globalState.battery_voltage * 1000.0f);
            telemetryPacket.rssi_uplink     = last_rssi; 
            
            xSemaphoreGive(stateMutex);
        }

        LoRaManager::sendTelemetry(telemetryPacket); 
        LoRa.receive(); // Volta a escutar
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_System_Mon(void *pvParameters) {
    esp_task_wdt_add(NULL); 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); 

    for(;;) {
        esp_task_wdt_reset(); 

        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            
            // Lê a bateria já limpa e estabilizada pela rotina de 10Hz
            float vbat_stable = globalState.battery_voltage;
            
            // [CORREÇÃO SPRINT 4]: Conflito Mestre-Escravo resolvido com a flag de Override
            if (vbat_stable < 6.4f && vbat_stable > 3.0f) {
                Serial.printf("ALERTA: BATERIA CRITICA! Forcando RTH (%.2f V)\n", vbat_stable);
                globalState.current_mode = MODE_RTH; 
                globalState.failsafe_override = true; // Tranca o modo. O piloto não consegue cancelar!
            }
            
            xSemaphoreGive(stateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 0: TAREFA DE LUZES NAV / STROBE (20Hz)
// ==========================================
void Task_Lights(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz (A cada 50ms)

    for(;;) {
        // 1. Puxa os dados vitais em segurança (Snapshot)
        float batt = 7.4f;
        bool armed = false;
        bool gps_ok = false;
        bool rth_on = false;
        
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            batt = globalState.battery_voltage;
            armed = globalState.is_armed;
            gps_ok = globalState.gps_fix;
            rth_on = (globalState.current_mode == MODE_RTH);
            xSemaphoreGive(stateMutex);
        }

        // Lógica para descobrir se o Home está travado sem violar o MissionManager
        // Se quisermos ser rigorosos, passamos o bool do Task_Navigation para o globalState.
        // Por agora, usamos a lógica: armado + com gps = travou o home
        bool is_home_locked = (armed && gps_ok); 

        // 2. Entrega para a máquina de estados dos LEDs
        LEDManager::update(batt, armed, gps_ok, rth_on, is_home_locked);

        // 3. Aguarda
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}