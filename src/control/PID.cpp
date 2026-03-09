#include "PID.h"

PID::PID(float p, float i, float d, float ff, float max_i, float max_out) {
    kp = p;
    ki = i;
    kd = d;
    kff = ff;
    max_integral = max_i;
    max_output = max_out;
    
    integral = 0.0f;
    prev_error = 0.0f;
    prev_derivative = 0.0f;
}

float PID::compute(float setpoint, float measurement, float dt, float tpa_factor) {
    // 1. PROTEÇÃO CRÍTICA DO RTOS
    // Se o escalonador do FreeRTOS reportar dt zero ou negativo, aborta o cálculo
    // para evitar divisão por zero (Hardware Panic) no termo derivativo.
    if (dt <= 0.0001f) return 0.0f; 

    // Cálculo do Erro atual
    float error = setpoint - measurement;

    // 2. APLICAÇÃO DO TPA (Throttle PID Attenuation)
    // Reduz a agressividade da correção em altas velocidades para evitar oscilações
    float current_kp = kp * tpa_factor;
    float current_kd = kd * tpa_factor;

    // 3. TERMO PROPORCIONAL
    float P = current_kp * error;

    // 4. TERMO INTEGRAL COM ANTI-WINDUP
    integral += error * dt;
    
    // Saturação da Integral (Evita que o erro acumule infinitamente se o avião for impedido de girar)
    if (integral > max_integral) integral = max_integral;
    else if (integral < -max_integral) integral = -max_integral;
    
    float I = ki * integral;

    // 5. TERMO DERIVATIVO COM FILTRO PASSA-BAIXA (PT1 Filter)
    // Calcula o derivativo cru
    float raw_derivative = (error - prev_error) / dt;
    
    // Aplica o filtro PT1 (First Order Low-Pass Filter)
    // Um alpha de ~0.5 a 250Hz cria um corte (cutoff) eficiente contra a vibração do motor.
    // Isso salva as engrenagens dos servos e evita o superaquecimento.
    float filter_alpha = 0.5f; 
    float filtered_derivative = prev_derivative + filter_alpha * (raw_derivative - prev_derivative);
    
    // Salva os estados para o próximo ciclo (4ms depois)
    prev_derivative = filtered_derivative;
    prev_error = error;

    // Aplica o ganho ao derivativo já filtrado e limpo
    float D = current_kd * filtered_derivative;

    // 6. TERMO FEEDFORWARD (Avanço de Ação Aerodinâmico)
    // Reage instantaneamente ao manche, ignorando o erro.
    float FF = kff * setpoint;

    // 7. SOMA TOTAL
    float output = P + I + D + FF;

    // 8. SATURAÇÃO DE SAÍDA (Output Clamp)
    // Garante que o PID nunca peça uma mixagem maior do que a Matriz de Elevons suporta
    if (output > max_output) output = max_output;
    else if (output < -max_output) output = -max_output;

    return output;
}

void PID::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
    prev_derivative = 0.0f; // Reseta também a memória do filtro
}