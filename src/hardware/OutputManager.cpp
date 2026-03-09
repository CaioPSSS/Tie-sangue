#include "OutputManager.h"

// Frequência padrão para Servos e ESCs clássicos
const uint32_t PWM_FREQUENCY = 50; 

// Resolução de 14 bits (Valores de duty cycle de 0 a 16383)
const uint8_t PWM_RESOLUTION = 14; 
const uint32_t MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION) - 1; // 16383

void OutputManager::init() {
    // Configura o Canal 0 (Servo Esquerdo)
    ledcSetup(CHANNEL_SERVO_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_SERVO_LEFT, CHANNEL_SERVO_LEFT);

    // Configura o Canal 1 (Servo Direito)
    ledcSetup(CHANNEL_SERVO_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_SERVO_RIGHT, CHANNEL_SERVO_RIGHT);

    // Configura o Canal 2 (ESC Motor)
    ledcSetup(CHANNEL_ESC, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_ESC_MOTOR, CHANNEL_ESC);

    // Garante inicialização segura (Tudo centralizado e motor desligado)
    writeServos(1500, 1500); 
    writeMotor(1000, false); // Começa desarmado
    
    Serial.println("Hardware de Saída (LEDC) Inicializado a 50Hz.");
}

// Converte a largura de pulso desejada (em microssegundos) para o Duty Cycle exigido pelo LEDC do ESP32
uint32_t OutputManager::usToDutyCycle(uint16_t microseconds) {
    // A 50Hz, um ciclo completo demora 20.000 microssegundos (20ms).
    // O valor do Duty Cycle (0 a 16383) é proporcional ao tempo ligado dentro desses 20ms.
    // Fórmula: (Microsegundos / 20.000) * 16383
    
    uint32_t duty = (uint32_t)microseconds * MAX_DUTY_CYCLE / 20000;
    return duty;
}

void OutputManager::writeServos(uint16_t left_us, uint16_t right_us) {
    // Proteção de segurança: Nunca enviar um sinal fora do padrão RC que pode fritar um servo
    left_us = constrain(left_us, 900, 2100);
    right_us = constrain(right_us, 900, 2100);

    ledcWrite(CHANNEL_SERVO_LEFT, usToDutyCycle(left_us));
    ledcWrite(CHANNEL_SERVO_RIGHT, usToDutyCycle(right_us));
}

void OutputManager::writeMotor(uint16_t throttle_us, bool is_armed) {
    if (!is_armed) {
        // Se a chave "Armar" estiver desligada, ignora qualquer comando do PID/Rádio
        // e tranca o motor no sinal de paragem absoluta.
        ledcWrite(CHANNEL_ESC, usToDutyCycle(1000));
        return;
    }

    // Proteção de segurança do ESC
    throttle_us = constrain(throttle_us, 1000, 2000);
    ledcWrite(CHANNEL_ESC, usToDutyCycle(throttle_us));
}