#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

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

// ==========================================
// FUNÇÃO GEODÉSICA: PRESSÃO -> ALTITUDE
// ==========================================
float computeAltitude(float pressure_pa, float sea_level_pa = 101325.0f) {
    // Se a pressão for inválida (sensor desconectado), evita cálculo bizarro
    if (pressure_pa <= 0) return 0.0f; 
    
    // Fórmula Barométrica Internacional
    return 44330.0f * (1.0f - pow(pressure_pa / sea_level_pa, 0.190295f));
}

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

    // ==========================================
    // INICIALIZAÇÃO DOS MÓDULOS DE HARDWARE
    // ==========================================
    LoRaManager::init();
    SensorManager::initSensors();
    OutputManager::init();
    GPSManager::init();
    BatteryManager::init();

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
        RawIMU imuData;
        SensorManager::readIMU(imuData);

        // 2. Atualizar AHRS (Filtro Mahony)
        ahrs.update(imuData.gx, imuData.gy, imuData.gz, imuData.ax, imuData.ay, imuData.az, dt);

        // ========================================================
        // 3. BUSCAR COMANDOS DO CORE 0 (Rádio Humano + Navegação Auto)
        // ========================================================
        float rc_roll = 0.0f, rc_pitch = 0.0f, rc_throttle = 1000.0f;
        float nav_roll = 0.0f, nav_pitch = 0.0f, nav_throttle = 0.0f;
        uint8_t current_mode = MODE_MANUAL;
        bool is_armed = false;
        
        // Timeout 0: Se o Core 0 estiver ocupado, não trava o Core 1. Usa os valores do ciclo passado.
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) { 
            // Inputs Humanos do Rádio LoRa
            rc_roll = globalState.rc_roll_cmd;
            rc_pitch = globalState.rc_pitch_cmd;
            rc_throttle = globalState.rc_throttle_pwm;
            current_mode = globalState.current_mode;
            is_armed = globalState.is_armed;
            
            // Inputs Autônomos (Gerados pelo L1 e TECS na Task_Navigation)
            nav_roll = globalState.desired_roll_cmd;
            nav_pitch = globalState.desired_pitch_cmd;
            nav_throttle = globalState.desired_throttle;
            
            // Grava os dados limpos da IMU para o Core 0 usar
            globalState.roll_deg = ahrs.roll;
            globalState.pitch_deg = ahrs.pitch;
            globalState.earth_z_accel = ahrs.earth_z_accel;
            xSemaphoreGive(stateMutex);
        }

        // ========================================================
        // 4. MÁQUINA DE ESTADOS (O Cérebro da Asa)
        // ========================================================
        // A FSM decide se ignora os PIDs (Modo Manual) ou se funde o L1/TECS
        fsm.update(current_mode, 
                   rc_roll, rc_pitch, rc_throttle,
                   nav_roll, nav_pitch, nav_throttle,
                   rollAnglePID, pitchAnglePID, rollRatePID, pitchRatePID);

        // ========================================================
        // 5. MALHAS DE CONTROLE PID E MIXER (Músculos)
        // ========================================================
        float mixer_roll_cmd = 0.0f;
        float mixer_pitch_cmd = 0.0f;

        if (fsm.bypass_pids) {
            // MODO MANUAL: Morte aos PIDs. Você está no controle direto (Pass-through).
            mixer_roll_cmd = fsm.final_roll_target;
            mixer_pitch_cmd = fsm.final_pitch_target;
        } else {
            // MODOS ESTABILIZADOS/AUTÔNOMOS: PIDs em Cascata assumem o comando.
            
            // Cálculo dinâmico do TPA com base no acelerador que a FSM escolheu usar
            float throttle_percent = constrain((fsm.final_throttle_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
            float tpa_factor = 1.0f;
            if (throttle_percent > 0.5f) {
                tpa_factor = 1.0f - ((throttle_percent - 0.5f) * 0.6f); 
            }

            // Malha Externa (Transforma o Ângulo alvo numa Rotação desejada)
            float desired_roll_rate = rollAnglePID.compute(fsm.final_roll_target, ahrs.roll, dt, 1.0f);
            float desired_pitch_rate = pitchAnglePID.compute(fsm.final_pitch_target, ahrs.pitch, dt, 1.0f);

            // Malha Interna (Luta contra a turbulência medindo a Rotação real do Giroscópio)
            mixer_roll_cmd = rollRatePID.compute(desired_roll_rate, imuData.gx, dt, tpa_factor);
            mixer_pitch_cmd = pitchRatePID.compute(desired_pitch_rate, imuData.gy, dt, tpa_factor);
        }

        // ========================================================
        // 6. MATRIZ DE ELEVONS E SAÍDA DE POTÊNCIA
        // ========================================================
        elevonMixer.compute(mixer_pitch_cmd, mixer_roll_cmd);

        // Envia o pulso elétrico para os Servos Esquerdo e Direito
        OutputManager::writeServos(elevonMixer.servo_left_pwm, elevonMixer.servo_right_pwm);

        // Envia energia para o Motor (Somente se a chave "Arm" do rádio estiver ativa)
        OutputManager::writeMotor(fsm.final_throttle_pwm, is_armed);
        
        // Cede o processamento e aguarda até fechar 4ms exatos
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 1: MALHA DE NAVEGAÇÃO (50Hz) - INTELIGÊNCIA
// ==========================================
void Task_Navigation(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz
    const float dt = 0.02f;

    for(;;) {
        // 1. Variáveis de Estado Local (Cópia segura do Core 1 e do GPS)
        float current_earth_z_accel = 0.0f;
        float current_ground_speed = 0.0f; 
        float current_lat = 0.0f;
        float current_lon = 0.0f;
        float current_course = 0.0f;
        bool  has_gps_fix = false;

        // Busca os dados mais recentes gerados pelas outras Tasks (Protegido por Mutex)
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            current_earth_z_accel = globalState.earth_z_accel;
            current_ground_speed = globalState.ground_speed_ms;
            
            // O Segredo aqui: Converte os inteiros do LoRa de volta para decimais puros para a Matemática!
            current_lat = globalState.lat / 10000000.0f;
            current_lon = globalState.lon / 10000000.0f;
            current_course = globalState.gps_course_deg;
            has_gps_fix = globalState.gps_fix;
            
            xSemaphoreGive(stateMutex);
        }

        // 2. LER SENSORES LENTOS EM SEGURANÇA (Barômetro)
        float baro_pressure, baro_temp;
        
        // 3. FUSÃO VERTICAL (Variômetro / Kalman 1D)
        // Lemos o barômetro E aplicamos os Sanity Checks construídos no SensorManager
        if (SensorManager::readBaro(baro_pressure, baro_temp)) {
            // Leitura perfeita. Converte a pressão limpa em metros
            float baro_alt = computeAltitude(baro_pressure);
            
            // Atualiza o filtro com a pressão do Core 0 e a aceleração G limpa do Core 1
            verticalFilter.update(baro_alt, current_earth_z_accel, dt);
        } else {
            // Failsafe de Hardware: O Barômetro falhou, desconectou ou entregou lixo
            // Passamos a própria estimativa antiga de volta para ele, 
            // forçando-o a confiar APENAS na integração inercial deste ciclo (Acelerômetro Z).
            verticalFilter.update(verticalFilter.estimated_altitude_m, current_earth_z_accel, dt);
        }


        // 3.1 LOCK DO HOME (Ponto de Retorno)
        static bool home_locked = false;
        
        // Variável local para saber se os motores foram armados
        bool is_armed_now = false; 
        if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            is_armed_now = globalState.is_armed;
            xSemaphoreGive(stateMutex);
        }

        // Condições vitais para travar o Ponto de Retorno:
        // 1. O Home ainda não foi gravado neste voo.
        // 2. O GPS tem sinal tridimensional validado.
        // 3. O piloto bateu a chave "Armar" no rádio LoRa (Decolagem iminente).
        if (!home_locked && has_gps_fix && is_armed_now) {
            
            // Grava a coordenada exata e a altitude local atual como Ponto Zero
            missionManager.setHome(current_lat, current_lon, verticalFilter.estimated_altitude_m);
            home_locked = true;
            
            Serial.println(">>> HOME POINT TRAVADO COM SUCESSO! <<<");
            
            // Dica de Hardware: Se você tiver um Buzzer no pino do ESP32, 
            // este é o momento de mandar fazer "BEEP BEEP BEEP" longo para 
            // avisar o piloto de que é seguro jogar a asa no ar!
        }

        // 4. ALVOS DE NAVEGAÇÃO (Waypoints)
        // Verifica o Deadman Switch (Failsafe)
        bool is_failsafe_active = (millis() - globalState.last_rc_packet_ms) > 1500;
        
        // Se a chave no PC pedir RTH ou o sinal de rádio cair, aciona a emergência
        bool force_rth = (globalState.current_mode == MODE_RTH) || is_failsafe_active;

        // 4. ATUALIZA A MÁQUINA DE MISSÃO / RTH
        missionManager.update(current_lat, current_lon, verticalFilter.estimated_altitude_m, force_rth);

        // Extrai os Alvos Dinâmicos do Gestor
        float wp_prev_lat   = missionManager.getPrevLat();
        float wp_prev_lon   = missionManager.getPrevLon();
        float wp_next_lat   = missionManager.getActiveLat();
        float wp_next_lon   = missionManager.getActiveLon();
        float target_alt    = missionManager.getActiveAltitude();
        float target_speed  = missionManager.getActiveSpeed();

        // 5. PROCESSAMENTO L1 E TECS
        if (has_gps_fix) {
            l1Guidance.compute(current_lat, current_lon, current_ground_speed, current_course, 
                               wp_prev_lat, wp_prev_lon, wp_next_lat, wp_next_lon);
        } else {
            l1Guidance.roll_cmd_deg = 0.0f; 
        }

        tecs.compute(target_alt, verticalFilter.estimated_altitude_m, 
                     verticalFilter.vertical_speed_ms, 
                     target_speed, current_ground_speed, dt);

        // 6. ENVIAR COMANDOS GERADOS PARA O CORE 1 E TELEMETRIA
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            globalState.desired_roll_cmd = l1Guidance.roll_cmd_deg;
            globalState.desired_pitch_cmd = tecs.pitch_cmd_deg;
            globalState.desired_throttle = tecs.throttle_cmd_percent;
            
            // Salva as altitudes limpas para o LoRa transmitir para o seu Notebook
            globalState.altitude_m = verticalFilter.estimated_altitude_m; 
            globalState.vertical_speed_ms = verticalFilter.vertical_speed_ms;
            
            xSemaphoreGive(stateMutex);
        }

        // Aguarda cravado para o RTOS manter a taxa de atualização de 50Hz exatos
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// CORE 0: TAREFAS DE COMUNICAÇÃO LENTA
// ==========================================
void Task_GPS_Parser(void *pvParameters) {
    for(;;) {
        // O update() lê a Serial2 de forma não bloqueante.
        // Se retornar true, significa que o NEO-6M acabou de calcular uma nova posição.
        if (GPSManager::update()) {
            
            // Só escrevemos no estado global se o FIX for válido 
            // (evita enviar dados de latitude 0,0 para o algoritmo de L1 Guidance)
            if (GPSManager::hasValidFix()) {
                
                // Bloqueia a memória rapidamente para atualizar a cartografia
                if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                    
                    globalState.lat = GPSManager::getLatitudeE7();
                    globalState.lon = GPSManager::getLongitudeE7();
                    globalState.ground_speed_ms = GPSManager::getSpeedMPS();
                    globalState.gps_course_deg = GPSManager::getCourseDeg();
                    globalState.gps_fix = true;
                    
                    xSemaphoreGive(stateMutex);
                    
                    // Nota de Debug: Se testar no exterior, quando os LEDs do NEO-6M piscarem,
                    // deverá ver coordenadas próximas de -1297XXXXX, -3850XXXXX (região de Salvador).
                }
            } else {
                // Se perdermos o sinal (ex: nuvens densas, interferência severa do VTX)
                if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                    globalState.gps_fix = false;
                    xSemaphoreGive(stateMutex);
                }
            }
        }
        
        // Cede tempo para o FreeRTOS cuidar da Task_LoRa_Comm e Task_Navigation do Core 0
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
        // 1. LER TENSÃO DA BATERIA (2S Li-ion)
        float current_vbat = BatteryManager::readVoltage();

        // 2. ESCREVER NO ESTADO GLOBAL PARA A TELEMETRIA
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            globalState.battery_voltage = current_vbat;
            xSemaphoreGive(stateMutex);
        }

        // 3. ALARME CRÍTICO DE BATERIA BAIXA
        // Li-ion 2S: Max 8.4V | Nominal 7.4V | Crítico 6.0V a 6.4V
        if (current_vbat < 6.4f) {
            Serial.printf("ALERTA: BATERIA CRÍTICA! (%.2f V)\n", current_vbat);
            // TODO: Quando tivermos a Máquina de Estados, aqui forçaremos o RTH!
        }

        // Aguarda fechar o ciclo de 1 segundo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}