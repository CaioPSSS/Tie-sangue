#include "BatteryManager.h"

float BatteryManager::filtered_voltage = 7.4f; // Inicia na tensão nominal de uma 2S

void BatteryManager::init() {
    pinMode(PIN_BATTERY_ADC, INPUT);
    // Configura a resolução do ADC para 12 bits (0 a 4095)
    analogReadResolution(12);
    // Atenuação de 11dB permite ler até ~3.3V no pino físico
    analogSetAttenuation(ADC_11db);
    
    Serial.println("Monitor de Bateria Inicializado.");
}

float BatteryManager::readVoltage() {
    // Lê o pino (A função analogReadMilliVolts do ESP32 já vem calibrada de fábrica!)
    uint32_t adc_mv = analogReadMilliVolts(PIN_BATTERY_ADC);
    
    // Converte os milivolts do pino para os Volts reais da Bateria
    float raw_voltage = (adc_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
    
    // Aplica um Filtro Passa-Baixa (PT1 Filter) leve
    // Se o motor puxar muita corrente de repente, a tensão cai por 1 segundo,
    // o filtro impede que o Failsafe ache que a bateria acabou instantaneamente.
    filtered_voltage = filtered_voltage * 0.9f + raw_voltage * 0.1f;
    
    return filtered_voltage;
}