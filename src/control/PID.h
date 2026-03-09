#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    // Ganhos base do Controlador
    float kp;
    float ki;
    float kd;
    float kff; // FeedForward (Avanço de Ação)

    // Limites de Segurança
    float max_integral;
    float max_output;

    PID(float p, float i, float d, float ff, float max_i, float max_out);

    // Calcula a saída do PID. 
    // dt: Delta time (segundos). 
    // tpa_factor: Atenuador baseado no motor (1.0 = normal, 0.5 = corta P e D pela metade)
    float compute(float setpoint, float measurement, float dt, float tpa_factor = 1.0f);

    // Reseta a memória do controlador (Usado ao trocar de modos de voo)
    void reset();

private:
    float integral;
    float prev_error;
    
    // Variável de estado para o filtro passa-baixa do termo Derivativo
    float prev_derivative; 
};

#endif