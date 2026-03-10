#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include <Arduino.h>

// --------------------------------------------------------
// MAPEAMENTO DE PINOS (Configuração da Asa Voadora)
// --------------------------------------------------------
// O ESP32 pode usar quase qualquer pino para PWM. Escolha pinos
// que não interfiram no Boot (evite 0, 2, 5, 12, 15).
#define PIN_SERVO_LEFT  13
#define PIN_SERVO_RIGHT 12
#define PIN_ESC_MOTOR   27

// --------------------------------------------------------
// CANAIS DE HARDWARE DO LEDC (ESP32)
// O ESP32 tem 16 canais independentes de PWM.
// --------------------------------------------------------
#define CHANNEL_SERVO_LEFT  0
#define CHANNEL_SERVO_RIGHT 1
#define CHANNEL_ESC         2

class OutputManager {
public:
    // Inicializa os pinos de PWM com frequências de aviação
    static void init();

    // Envia o sinal para os Servos dos Elevons (1000 a 2000 microssegundos)
    static void writeServos(uint16_t left_us, uint16_t right_us);

    // Envia o sinal para o ESC do Motor (1000 a 2000 microssegundos)
    // Apenas envia se a asa estiver Armada, caso contrário desliga o motor.
    static void writeMotor(uint16_t throttle_us, bool is_armed);
    
private:
    // Funções auxiliares para converter microssegundos no ciclo de trabalho do LEDC
    static uint32_t usToDutyCycle(uint16_t microseconds);
};

#endif