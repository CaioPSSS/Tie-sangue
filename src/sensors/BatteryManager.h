#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>

// Pino Analógico (ADC) do ESP32 onde o divisor de tensão está ligado
// Dica: Use os pinos ADC1 (32 a 36), pois o ADC2 conflita com o Wi-Fi (caso usássemos)
#define PIN_BATTERY_ADC 34 

class BatteryManager {
public:
    static void init();
    
    // Lê e filtra a tensão da bateria em Volts
    static float readVoltage();

private:
    // Fator do Divisor de Tensão. 
    // Ex: R1 = 10k, R2 = 4.7k -> Fator = (10 + 4.7) / 4.7 = 3.127
    static constexpr float VOLTAGE_DIVIDER_RATIO = 3.127f;
    
    // Filtro Passa-Baixa para ignorar picos rápidos de queda de tensão (ruído do motor)
    static float filtered_voltage;
};

#endif