#include "TECS.h"

TECS::TECS() {
    throttle_cmd_percent = 0.0f;
    pitch_cmd_deg = 0.0f;
}

void TECS::compute(float target_alt, float current_alt, float v_speed, 
                   float target_speed, float current_speed, float dt) {
                   
    // ==========================================================
    // 1. CÁLCULO NORMAL DA MALHA DE ENERGIA (TECS BASE)
    // ==========================================================
    // Taxa de Erro de Energia Potencial (Diferença de Altitude convertida em taxa de subida)
    float alt_error = target_alt - current_alt;
    float target_v_speed = constrain(alt_error * 0.5f, -5.0f, 5.0f); // Limita subida/descida a 5m/s
    
    // Ponto D (Potencial): Taxa desejada vs Taxa real (v_speed vem do Filtro Vertical)
    float Ep_dot_error = target_v_speed - v_speed;

    // Taxa de Erro de Energia Cinética
    float speed_error = target_speed - current_speed;
    float speed_dot = (current_speed - prev_speed) / dt; // Aceleração longitudinal real
    prev_speed = current_speed;
    
    // Normaliza a aceleração cinética para combinar com a energia potencial (dividindo por G)
    float target_speed_dot = constrain(speed_error * 0.2f, -2.0f, 2.0f); 
    float Ek_dot_error = ((current_speed * target_speed_dot) / GRAVITY) - 
                         ((current_speed * speed_dot) / GRAVITY);

    // FUSÃO TECS (Energia Total e Balanço)
    float Total_Energy_Error = Ep_dot_error + Ek_dot_error;     // Pede mais ou menos Motor
    float Energy_Balance_Error = Ep_dot_error - Ek_dot_error;   // Pede mais ou menos Arfagem (Pitch)

    // CONTROLADOR PI DE ACELERAÇÃO (Throttle Normal)
    throttle_integ += Total_Energy_Error * throttle_Ki * dt;
    throttle_integ = constrain(throttle_integ, 0.0f, 1.0f); // Anti-windup motor

    float raw_throttle = (Total_Energy_Error * throttle_Kp) + throttle_integ;
    float cruise_throttle = 0.4f; // Motor de cruzeiro base (40%)
    float normal_throttle_cmd = constrain(raw_throttle + cruise_throttle, 0.0f, 1.0f);

    // CONTROLADOR PI DE ARFAGEM (Pitch Normal)
    pitch_integ += Energy_Balance_Error * pitch_Ki * dt;
    pitch_integ = constrain(pitch_integ, min_pitch_deg, max_pitch_deg); // Anti-windup pitch

    float raw_pitch = (Energy_Balance_Error * pitch_Kp) + pitch_integ;
    float normal_pitch_cmd = constrain(raw_pitch, min_pitch_deg, max_pitch_deg);

    // ==========================================================
    // 2. SISTEMA ANTI-ESTOL COM HISTERESE PROPORCIONAL (SPRINT 3)
    // ==========================================================
    // Em vez do comportamento "Bang-Bang" destrutivo, criamos um Fade-In suave
    
    float emergency_blend = 0.0f; // 0.0 = Navegação Normal | 1.0 = Recuperação de Estol 100%

    if (current_speed <= min_airspeed) {
        emergency_blend = 1.0f; // Estol absoluto: Força a recuperação imediata
    } 
    else if (current_speed < safe_airspeed) {
        // Interpolação linear: Cria uma rampa suave entre os 12 m/s e os 10 m/s
        emergency_blend = (safe_airspeed - current_speed) / (safe_airspeed - min_airspeed);
    }

    // Mistura progressiva final entre a navegação pacífica e o mergulho agressivo de recuperação
    pitch_cmd_deg = (normal_pitch_cmd * (1.0f - emergency_blend)) + (min_pitch_deg * emergency_blend);
    throttle_cmd_percent = (normal_throttle_cmd * (1.0f - emergency_blend)) + (1.0f * emergency_blend);
}