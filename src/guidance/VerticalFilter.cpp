#include "VerticalFilter.h"

VerticalFilter::VerticalFilter() {
    estimated_altitude_m = 0.0f;
    vertical_speed_ms = 0.0f;
    accel_bias = 0.0f;
}

void VerticalFilter::update(float baro_alt_m, float earth_z_accel_g, float dt) {
    
    // 1. Converter a aceleração de G's para m/s² e subtrair o Bias estimado
    // (Lembrando que o Core 1 já removeu 1G da gravidade, então earth_z_accel_g 
    // deve ser ~0.0 quando a asa está a voar nivelada sem subir nem descer)
    float accel_z_ms2 = (earth_z_accel_g * GRAVITY) - accel_bias;

    // 2. PASSO DE PREDIÇÃO (Cinemática Básica)
    // Aceleração altera a velocidade. Velocidade altera a posição.
    estimated_altitude_m += vertical_speed_ms * dt;
    vertical_speed_ms += accel_z_ms2 * dt;

    // 3. PASSO DE CORREÇÃO (Fundir com o Barómetro)
    // Calcula a diferença entre onde o Barómetro diz que estamos e onde a Matemática diz que estamos
    float alt_error = baro_alt_m - estimated_altitude_m;

    // Aplica as correções aos nossos estados com base nos ganhos de confiança
    estimated_altitude_m += Kp_alt * alt_error * dt;
    vertical_speed_ms += Kp_vel * alt_error * dt;
    
    // O pulo do gato: Se a matemática erra consistentemente para o mesmo lado,
    // significa que o acelerómetro está com um bias térmico. O integrador corrige isso.
    accel_bias -= Ki_accel * alt_error * dt;
}