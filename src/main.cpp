#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "common/SharedTypes.h"

// ==========================================
// VARIÁVEIS GLOBAIS DE IPC (Inter-Process Comm)
// ==========================================
FlightState globalState;
SemaphoreHandle_t stateMutex;          // Protege a escrita/leitura do globalState
QueueHandle_t telemetryQueue;          // Fila para enviar dados para a Task LoRa

// ==========================================
// DECLARAÇÃO DAS TAREFAS (Conforme Masterplan)
// ==========================================
// CORE 1 (Piloto)
void Task_FlightLoop(void *pvParameters);
void Task_Navigation(void *pvParameters);

// CORE 0 (Comunicações)
void Task_GPS_Parser(void *pvParameters);
void Task_LoRa_Comm(void *pvParameters);
void Task_System_Mon(void *pvParameters);

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
    Serial.begin(115200);
    Serial.println("INICIANDO SISTEMA DE CONTROLE - ASA VOADORA V1.0");

    // Inicialização dos Primitivos de Sincronização do FreeRTOS
    stateMutex = xSemaphoreCreateMutex();
    telemetryQueue = xQueueCreate(5, sizeof(PacketTelemetryLoRa_t)); // Buffer de 5 pacotes

    if (stateMutex == NULL || telemetryQueue == NULL) {
        Serial.println("ERRO CRÍTICO: Falha ao alocar memória para o RTOS.");
        while(1); // Trava o sistema
    }

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 1 (Prioridade Máxima) ---
    // Task_FlightLoop: 250Hz - Lê MPU6050, AHRS, PID, Elevons
    xTaskCreatePinnedToCore(
        Task_FlightLoop, "FlightLoop", 8192, NULL, 5, NULL, 1);

    // Task_Navigation: 50Hz - TECS, L1 Guidance, Altitude
    xTaskCreatePinnedToCore(
        Task_Navigation, "NavLoop", 4096, NULL, 4, NULL, 1);

    // --- INSTANCIAÇÃO DAS TAREFAS NO CORE 0 (Comunicação/IO) ---
    // Task_GPS_Parser: Assíncrono - Lê UART do NEO-6M
    xTaskCreatePinnedToCore(
        Task_GPS_Parser, "GPSParser", 4096, NULL, 3, NULL, 0);

    // Task_LoRa_Comm: 10Hz - Envia telemetria via SPI
    xTaskCreatePinnedToCore(
        Task_LoRa_Comm, "LoRaComm", 4096, NULL, 3, NULL, 0);

    // Task_System_Mon: 1Hz - Bateria, Watchdog e Failsafe
    xTaskCreatePinnedToCore(
        Task_System_Mon, "SysMon", 2048, NULL, 1, NULL, 0);

    // Deleta a task padrão do Arduino (void loop) para liberar memória, 
    // já que faremos tudo estritamente no FreeRTOS.
    vTaskDelete(NULL); 
}

void loop() {
    // Vazio. O FreeRTOS comanda a partir do setup.
}

// ==========================================
// IMPLEMENTAÇÃO DOS ESQUELETOS DAS TAREFAS
// ==========================================

void Task_FlightLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250 Hz (4ms)

    for(;;) {
        // 1. Ler Sensores Rápidos (MPU6050 I2C Non-Blocking)
        // 2. Rodar Filtro Complementar (AHRS Quaternions)
        // 3. Rodar PIDs de Atitude e Rate
        // 4. Calcular Matriz de Elevons (Dual Clamp)
        // 5. Enviar PWM pros Servos
        
        // Aguarda exatamente o tempo restante para fechar 4ms (Determinismo)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_Navigation(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz (20ms)

    for(;;) {
        // 1. Executar Algoritmo L1 Guidance (Cross-Track Error)
        // 2. Executar TECS (Controle de Energia Total)
        // 3. Checar Estado do RTH
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_GPS_Parser(void *pvParameters) {
    for(;;) {
        // 1. Ficar escutando a UART (Serial2) bloqueante/assíncrona
        // 2. Quando formar uma sentença NMEA válida, decodificar
        // 3. xSemaphoreTake(stateMutex) -> Atualizar Lat/Lon/COG -> xSemaphoreGive()
        vTaskDelay(pdMS_TO_TICKS(10)); // Cede tempo pro Core 0 respirar
    }
}

void Task_LoRa_Comm(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz (100ms)

    PacketTelemetryLoRa_t telemetryPacket;

    for(;;) {
        // 1. Construir a struct compactada lendo dados do globalState (via Mutex)
        // 2. Calcular Checksum CRC8
        // 3. Disparar via SPI para o SX1278
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_System_Mon(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 Hz (1000ms)

    for(;;) {
        // 1. Ler tensão da bateria (ADC)
        // 2. Checar tempo do último pacote do Rádio (Gatilho de Failsafe)
        // 3. Checar stack highwater mark (segurança de memória)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}