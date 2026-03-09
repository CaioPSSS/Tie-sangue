#ifndef VERTICAL_FILTER_H
#define VERTICAL_FILTER_H

#include <Arduino.h>

class VerticalFilter {
public:
    // Ganhos do Filtro (Sintonia fina para a Asa Voadora)
    float Kp_alt = 2.0f;    // Peso dado ao Barómetro para corrigir a Altitude
    float Kp_vel = 1.0f;    // Peso dado ao Barómetro para corrigir a Velocidade Vertical
    float Ki_accel = 0.05f; // Peso para descobrir e corrigir o erro (bias) do Acelerómetro

    // Saídas limpas e sem atraso (Prontas para o TECS e Telemetria)
    float estimated_altitude_m;
    float vertical_speed_ms;

    VerticalFilter();

    // Executa o filtro a 50Hz no Core 0
    // baro_alt_m: Altitude crua do BMP280 (metros)
    // earth_z_accel: Aceleração Z Inercial vinda do Core 1 (g's, onde 0 = parado)
    // dt: Delta time em segundos (ex: 0.02 para 50Hz)
    void update(float baro_alt_m, float earth_z_accel_g, float dt);

private:
    const float GRAVITY = 9.81f;
    
    // Estado interno para correção de deriva térmica
    float accel_bias;
};

#endif