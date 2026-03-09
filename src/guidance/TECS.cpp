#include "TECS.h"

TECS::TECS() {
    throttle_cmd_percent = 0.0f;
    pitch_cmd_deg = 0.0f;
}

void TECS::compute(float target_alt, float current_alt, float v_speed, 
                   float target_speed, float current_speed, float dt) {
                   
    // 1. PROTEÇÃO DE ESTOL (Stall Prevention - Emergência Máxima)
    // Se a velocidade cair abaixo do mínimo absoluto, o TECS toma uma atitude drástica:
    // Abandona a tentativa de subir, abaixa o nariz e dá motor máximo.
    if (current_speed < min_airspeed) {
        throttle_cmd_percent = 1.0f; // 100% Motor
        pitch_cmd_deg = min_pitch_deg; // Pica o nariz para recuperar fluxo de ar
        return; 
    }

    // 2. CÁLCULO DAS TAXAS DE ENERGIA ATUAIS E DESEJADAS
    // Taxa de Erro de Energia Potencial (Diferença de Altitude convertida em taxa de subida)
    float alt_error = target_alt - current_alt;
    float target_v_speed = constrain(alt_error * 0.5f, -5.0f, 5.0f); // Limita subida/descida a 5m/s
    
    // Ponto D (Potencial): Taxa desejada vs Taxa real (v_speed vem do Filtro de Kalman Vertical)
    float Ep_dot_error = target_v_speed - v_speed;

    // Taxa de Erro de Energia Cinética
    float speed_error = target_speed - current_speed;
    float speed_dot = (current_speed - prev_speed) / dt; // Aceleração longitudinal real
    prev_speed = current_speed;
    
    // Normaliza a aceleração cinética para combinar com a energia potencial (dividindo por G)
    float target_speed_dot = constrain(speed_error * 0.2f, -2.0f, 2.0f); 
    float Ek_dot_error = ((current_speed * target_speed_dot) / GRAVITY) - 
                         ((current_speed * speed_dot) / GRAVITY);

    // 3. FUSÃO TECS (Energia Total e Balanço)
    // Erro de Energia Total: Pede mais ou menos Motor
    float Total_Energy_Error = Ep_dot_error + Ek_dot_error;
    
    // Erro de Balanço de Energia: Pede mais ou menos Arfagem (Pitch)
    float Energy_Balance_Error = Ep_dot_error - Ek_dot_error;

    // 4. CONTROLADOR PI DE ACELERAÇÃO (Throttle)
    throttle_integ += Total_Energy_Error * throttle_Ki * dt;
    throttle_integ = constrain(throttle_integ, 0.0f, 1.0f); // Anti-windup motor (0 a 100%)

    float raw_throttle = (Total_Energy_Error * throttle_Kp) + throttle_integ;
    
    // Adiciona o "Cruise Throttle" (Motor de cruzeiro esperado, ex: 40%) como feedforward
    float cruise_throttle = 0.4f; 
    throttle_cmd_percent = constrain(raw_throttle + cruise_throttle, 0.0f, 1.0f);

    // 5. CONTROLADOR PI DE ARFAGEM (Pitch)
    pitch_integ += Energy_Balance_Error * pitch_Ki * dt;
    pitch_integ = constrain(pitch_integ, min_pitch_deg, max_pitch_deg); // Anti-windup pitch

    float raw_pitch = (Energy_Balance_Error * pitch_Kp) + pitch_integ;
    pitch_cmd_deg = constrain(raw_pitch, min_pitch_deg, max_pitch_deg);
}