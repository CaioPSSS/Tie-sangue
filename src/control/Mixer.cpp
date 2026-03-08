#include "Mixer.h"

ElevonMixer::ElevonMixer() {
    servo_left_pwm = PWM_CENTER;
    servo_right_pwm = PWM_CENTER;
}

void ElevonMixer::compute(float pitch_cmd, float roll_cmd) {
    // 1. LIMITADOR DE SOBREVIVÊNCIA (Pitch Priority Clamp)
    // O pitch não pode ultrapassar o curso máximo físico mecânico (500us)
    int16_t pitch = (int16_t)pitch_cmd;
    if (pitch > MAX_TRAVEL) pitch = MAX_TRAVEL;
    if (pitch < -MAX_TRAVEL) pitch = -MAX_TRAVEL;

    // 2. CÁLCULO DA MARGEM RESTANTE
    // Descobrimos quanto curso sobrou no servo depois do pitch pegar a parte dele
    int16_t available_roll_travel = MAX_TRAVEL - abs(pitch);

    // 3. COMPRESSÃO DA ROLAGEM (Roll Clamp)
    // A rolagem só pode usar o espaço que sobrou.
    int16_t roll = (int16_t)roll_cmd;
    if (roll > available_roll_travel) roll = available_roll_travel;
    if (roll < -available_roll_travel) roll = -available_roll_travel;

    // 4. MATRIZ GEOMÉTRICA DE MIXAGEM
    // Por convenção aeronáutica:
    // Pitch Positivo (Nariz para cima) -> Ambos os elevons sobem (+)
    // Roll Positivo (Curva para direita) -> Elevon Esquerdo desce (-), Direito sobe (+)
    int16_t out_left  = pitch - roll;
    int16_t out_right = pitch + roll;

    // 5. INVERSÃO DE HARDWARE E TRIMS MECÂNICOS
    if (invert_left)  out_left  = -out_left;
    if (invert_right) out_right = -out_right;

    out_left  += trim_left;
    out_right += trim_right;

    // 6. GERAÇÃO DO PWM FINAL COM SATURAÇÃO DE SEGURANÇA (Hard Limit)
    int16_t final_left_pwm = PWM_CENTER + out_left;
    int16_t final_right_pwm = PWM_CENTER + out_right;

    // Garantia final contra envio de sinal fora do padrão do servo
    servo_left_pwm  = constrain(final_left_pwm, PWM_MIN, PWM_MAX);
    servo_right_pwm = constrain(final_right_pwm, PWM_MIN, PWM_MAX);
}