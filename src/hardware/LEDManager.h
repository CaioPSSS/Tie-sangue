#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <Arduino.h>

// Pinos físicos dos LEDs (Mude para os pinos que for usar no ESP32)
// Dica: Use pinos que não interfiram com os motores ou I2C (ex: 2, 4, 15)
#define PIN_LED_LEFT  2   // Vermelho (Norma)
#define PIN_LED_RIGHT 4   // Verde (Norma)
#define PIN_LED_TAIL  15  // Branco

class LEDManager {
public:
    static void init();
    
    // Função chamada pela Task a 20Hz (A cada 50ms)
    static void update(float battery_v, bool is_armed, bool has_gps, bool rth_active, bool home_locked);

private:
    static uint32_t tick_counter; // Conta os ciclos para criar animações
    static bool prev_home_locked;
    static uint32_t home_celebration_ticks; // Duração do aviso visual do Home
};

#endif