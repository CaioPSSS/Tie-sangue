#ifndef L1_GUIDANCE_H
#define L1_GUIDANCE_H

#include <Arduino.h>

class L1Guidance {
public:
    // Parâmetros de Ajuste (Tuning)
    float l1_period = 15.0f;       // Período de amortecimento em segundos (Define a agressividade)
    float l1_damping = 0.75f;      // Fator de amortecimento (0.75 a 0.85 é ideal para asas voadoras)
    float max_roll_angle = 45.0f;  // Limite máximo de inclinação em graus para segurança

    // Saída principal para o Core 1 (Pitch/Roll PIDs)
    float roll_cmd_deg;

    L1Guidance();

    // Executa a malha de navegação a 50Hz (Core 0)
    // Coordenadas devem estar em formato decimal (Lat/Lon)
    void compute(float current_lat, float current_lon, float ground_speed, float current_course,
                 float wp_prev_lat, float wp_prev_lon, float wp_next_lat, float wp_next_lon);

private:
    const float GRAVITY = 9.81f;
    const float EARTH_RADIUS = 6371000.0f; // Metros

    // Funções auxiliares de geodésia (Fórmula de Haversine)
    float getDistance(float lat1, float lon1, float lat2, float lon2);
    float getBearing(float lat1, float lon1, float lat2, float lon2);
};

#endif