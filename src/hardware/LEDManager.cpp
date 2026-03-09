#include "LEDManager.h"

uint32_t LEDManager::tick_counter = 0;
bool LEDManager::prev_home_locked = false;
uint32_t LEDManager::home_celebration_ticks = 0;

void LEDManager::init() {
    pinMode(PIN_LED_LEFT, OUTPUT);
    pinMode(PIN_LED_RIGHT, OUTPUT);
    pinMode(PIN_LED_TAIL, OUTPUT);
    
    // Começa com tudo ligado (Teste de lâmpadas)
    digitalWrite(PIN_LED_LEFT, HIGH);
    digitalWrite(PIN_LED_RIGHT, HIGH);
    digitalWrite(PIN_LED_TAIL, HIGH);
    
    Serial.println("Luzes de Navegacao (V-Tail) Inicializadas.");
}

void LEDManager::update(float battery_v, bool is_armed, bool has_gps, bool rth_active, bool home_locked) {
    tick_counter++;

    // ==========================================
    // 1. EVENTO ÚNICO: CELEBRAÇÃO DO HOME POINT
    // ==========================================
    // Deteta a transição de "Sem Home" para "Home Travado"
    if (home_locked && !prev_home_locked) {
        home_celebration_ticks = 60; // 60 ticks = 3 segundos de celebração
        prev_home_locked = true;
    }

    if (home_celebration_ticks > 0) {
        home_celebration_ticks--;
        // Pisca todos os LEDs juntos a cada 150ms (3 ticks)
        bool flash = (tick_counter % 6) < 3;
        digitalWrite(PIN_LED_LEFT, flash);
        digitalWrite(PIN_LED_RIGHT, flash);
        digitalWrite(PIN_LED_TAIL, flash);
        return; // Ignora o resto enquanto estiver a celebrar
    }

    // ==========================================
    // 2. HIERARQUIA DE ALERTAS CONTÍNUOS
    // ==========================================

    // ALERTA 1: Bateria Crítica (Morte iminente - Pisca muito rápido)
    if (battery_v > 3.0f && battery_v < 6.4f) {
        bool fast_blink = (tick_counter % 4) < 2; // Liga 100ms, desliga 100ms
        digitalWrite(PIN_LED_LEFT, fast_blink);
        digitalWrite(PIN_LED_RIGHT, fast_blink);
        digitalWrite(PIN_LED_TAIL, fast_blink);
        return;
    }

    // ALERTA 2: Bateria Baixa (Aviso de regresso - Pisca lento)
    if (battery_v >= 6.4f && battery_v < 7.0f) {
        bool slow_blink = (tick_counter % 20) < 10; // Liga 500ms, desliga 500ms
        digitalWrite(PIN_LED_LEFT, slow_blink);
        digitalWrite(PIN_LED_RIGHT, slow_blink);
        digitalWrite(PIN_LED_TAIL, slow_blink);
        return;
    }

    // ALERTA 3: RTH Acionado (Efeito "Polícia" alternado)
    if (rth_active) {
        bool left_phase = (tick_counter % 10) < 5;
        digitalWrite(PIN_LED_LEFT, left_phase);
        digitalWrite(PIN_LED_RIGHT, !left_phase); // Inverso
        digitalWrite(PIN_LED_TAIL, left_phase);
        return;
    }

    // ==========================================
    // 3. VOO NORMAL (PADRÃO AVIAÇÃO FAA)
    // ==========================================
    
    // Luzes das asas (Nav Lights) ficam SEMPRE acesas em modo normal
    digitalWrite(PIN_LED_LEFT, HIGH);
    digitalWrite(PIN_LED_RIGHT, HIGH);

    // Efeito Strobe da Cauda (Branco)
    if (is_armed) {
        // Armado: Duplo Strobe violento tipo Airbus (Pisca-Pisca... Pausa...)
        // Ciclo de 20 ticks (1 segundo)
        uint32_t step = tick_counter % 20;
        if (step == 0 || step == 2) {
            digitalWrite(PIN_LED_TAIL, HIGH); // Flashes curtos de 50ms
        } else {
            digitalWrite(PIN_LED_TAIL, LOW);
        }
    } else {
        // Desarmado (No chão): Pisca lento 1 vez por segundo
        bool tail_idle = (tick_counter % 20) < 2; // Fica ligado só 10% do tempo
        digitalWrite(PIN_LED_TAIL, tail_idle);
    }

    // (Opcional): Se não tiver GPS (has_gps == false), poderia desligar o LED das asas
    // brevemente a cada 2 segundos para o piloto saber que está "cego".
}