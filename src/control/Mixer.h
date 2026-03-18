#ifndef MIXER_H
#define MIXER_H

#include <Arduino.h>

class ElevonMixer {
public:
    // Configurações de Hardware (Definidas no setup)
    bool invert_left = false;
    bool invert_right = false;
    
    // Trims mecânicos em microssegundos (usados para centralizar ou dar reflexo de perfil)
    int16_t trim_left = 40; // Criar perfil reflex artificial
    int16_t trim_right = 40; // Criar perfil reflex artificial

    // Saídas finais prontas para a biblioteca ESP32Servo (em microssegundos)
    uint16_t servo_left_pwm;
    uint16_t servo_right_pwm;

    ElevonMixer();

    // Aplica a matriz com Saturação Aninhada (Dual Clamp)
    // pitch_cmd e roll_cmd devem vir do PID (idealmente de -500 a +500)
    void compute(float pitch_cmd, float roll_cmd);

private:
    // Limites absolutos do sinal PWM para proteger o servo
    const int16_t PWM_MIN = 1000;
    const int16_t PWM_MAX = 2000;
    const int16_t PWM_CENTER = 1500;
    const int16_t MAX_TRAVEL = 500; // Máxima deflexão a partir do centro
};

#endif