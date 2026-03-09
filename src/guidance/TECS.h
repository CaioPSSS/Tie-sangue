#ifndef TECS_H
#define TECS_H

#include <Arduino.h>

class TECS {
public:
    // Limites de Segurança Aerodinâmica (Tuning)
    float min_airspeed = 10.0f;     // m/s (Velocidade de Estol - Emergência Máxima)
    float safe_airspeed = 12.0f;    // m/s (Início da transição suave anti-estol)
    float cruise_airspeed = 15.0f;  // m/s (Velocidade de máxima eficiência)
    float max_airspeed = 25.0f;     // m/s (Limite estrutural)
    
    float max_pitch_deg = 20.0f;    // Nariz para cima (Evita estol de atitude)
    float min_pitch_deg = -15.0f;   // Nariz para baixo (Mergulho de recuperação)

    // Ganhos dos Controladores PI do TECS
    float throttle_Kp = 0.5f, throttle_Ki = 0.1f;
    float pitch_Kp = 0.5f, pitch_Ki = 0.05f;

    // Saídas para o Core 1 (Mixer e Motor)
    float throttle_cmd_percent; // 0.0 a 1.0 (0 a 100%)
    float pitch_cmd_deg;        // Comando de arfagem para o Angle PID

    TECS();

    // Executa a malha do TECS a 50Hz (Core 0)
    void compute(float target_alt, float current_alt, float v_speed, 
                 float target_speed, float current_speed, float dt);

private:
    const float GRAVITY = 9.81f;

    // Integradores
    float throttle_integ = 0.0f;
    float pitch_integ = 0.0f;
    
    float prev_speed = 0.0f;
};

#endif