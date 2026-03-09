#include "FlightModeManager.h"

FlightModeManager::FlightModeManager() {
    previous_mode = MODE_MANUAL;
    bypass_pids = true;
    final_roll_target = 0.0f;
    final_pitch_target = 0.0f;
    final_throttle_pwm = 1000.0f;
    smoothed_roll_target = 0.0f;
    smoothed_pitch_target = 0.0f;
}

void FlightModeManager::update(uint8_t current_mode, 
                               float rc_roll, float rc_pitch, float rc_throttle,
                               float nav_roll, float nav_pitch, float nav_throttle,
                               float dt,
                               PID &rollAnglePID, PID &pitchAnglePID,
                               PID &rollRatePID, PID &pitchRatePID) {
    
    // 1. DETEÇÃO DE TRANSIÇÃO (Prevenção de "Coice" Aerodinâmico)
    if (current_mode != previous_mode) {
        Serial.printf("FSM: Transicao de Modo [%d] para [%d]\n", previous_mode, current_mode);
        
        // Zera a memória de todos os PIDs para garantir uma transição suave
        rollAnglePID.reset();
        pitchAnglePID.reset();
        rollRatePID.reset();
        pitchRatePID.reset();
        
        previous_mode = current_mode;
    }

    // Variáveis alvo cruas (o que o algoritmo/rádio quer fazer instantaneamente)
    float raw_roll_target = 0.0f;
    float raw_pitch_target = 0.0f;
    
    // Constante de tempo do filtro (0.5 segundos para a transição completa)
    float time_constant = 0.5f;

    // 2. ÁRVORE DE DECISÃO (Switch-Case)
    switch (current_mode) {

        case MODE_MANUAL:
            // "Voo Livre": Supressão temporária e contínua de todo software PID e IMU.
            // Os valores dos sticks vão diretamente para a Matriz Elevon Mixing.
            bypass_pids = true;
            raw_roll_target = (rc_roll / 45.0f) * 500.0f; 
            raw_pitch_target = (rc_pitch / 45.0f) * 500.0f;
            final_throttle_pwm = rc_throttle;
            time_constant = 0.0f; // Sem atrasos em modo puramente manual!
            break;

        case MODE_ANGLE:
            // "Voo Assistido": PID Completo (Angle + Rate). 
            // O humano controla o ângulo desejado. Soltar o stick nivela a asa.
            bypass_pids = false;
            raw_roll_target = rc_roll;
            raw_pitch_target = rc_pitch;
            final_throttle_pwm = rc_throttle; // Acelerador ainda é humano
            time_constant = 0.1f; // Transição muito rápida para resposta do piloto
            break;

        case MODE_HOLD:
            // "Heading e Altitude": Híbrido semiautônomo.
            bypass_pids = false;
            // Se o piloto tocar nos sticks (deadband > 3 graus), ele assume o controle momentaneamente.
            if (abs(rc_roll) > 3.0f || abs(rc_pitch) > 3.0f) {
                raw_roll_target = rc_roll;
                raw_pitch_target = rc_pitch;
                final_throttle_pwm = rc_throttle;
                time_constant = 0.1f; // Piloto no controlo
            } else {
                // Se soltar os sticks, o TECS e L1 (Hold) assumem o controle de atitude
                raw_roll_target = nav_roll;
                raw_pitch_target = nav_pitch;
                // No modo Hold puro, o acelerador é automático (TECS)
                // Converte a %. do TECS (0.0 a 1.0) para PWM (1000 a 2000)
                final_throttle_pwm = 1000.0f + (nav_throttle * 1000.0f); 
                time_constant = 0.5f; // [CORREÇÃO SPRINT 3]: Robô assume suavemente
            }
            break;

        case MODE_AUTO:
        case MODE_RTH:
            // "Missão / Return To Home": Total Autonomia Robótica.
            // O humano é completamente suprimido (exceto pela chave de desarmar).
            // O Core 0 (Navegador) assume 100% da autoridade dos targets.
            bypass_pids = false;
            raw_roll_target = nav_roll;
            raw_pitch_target = nav_pitch;
            final_throttle_pwm = 1000.0f + (nav_throttle * 1000.0f);
            time_constant = 0.5f; // Movimentos fluídos do piloto automático
            break;

        default:
            // Failsafe Catcher: Estado desconhecido? Força Angle mode macio.
            bypass_pids = false;
            raw_roll_target = 0.0f;
            raw_pitch_target = 0.0f;
            final_throttle_pwm = rc_throttle;
            time_constant = 0.1f;
            break;
    }

    // ==========================================================
    // 3. APLICAÇÃO DO FILTRO EXPONENCIAL (SLEW RATE LIMITER)
    // ==========================================================
    if (time_constant > 0.0f && dt > 0.0f) {
        // Fórmula de suavização exponencial para evitar degraus matemáticos abruptos
        float alpha = dt / (time_constant + dt);
        smoothed_roll_target += (raw_roll_target - smoothed_roll_target) * alpha;
        smoothed_pitch_target += (raw_pitch_target - smoothed_pitch_target) * alpha;
    } else {
        // Passagem direta (Bypass do filtro no Modo Manual para resposta instantânea)
        smoothed_roll_target = raw_roll_target;
        smoothed_pitch_target = raw_pitch_target;
    }

    // Entrega o valor suavizado para os PIDs!
    final_roll_target = smoothed_roll_target;
    final_pitch_target = smoothed_pitch_target;
}