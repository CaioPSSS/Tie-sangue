#ifndef L1_GUIDANCE_H
#define L1_GUIDANCE_H

#include <Arduino.h>

class L1Guidance {
public:
    // [ATUALIZAÇÃO FINAL] Parâmetros Anti-Tip Stall para Asa de 61cm
    float l1_period = 15.0f;       // Aumentado (Curvas majestosas e bem abertas)
    float l1_damping = 0.85f;      // Fator de amortecimento máximo
    float max_roll_angle = 35.0f;  // Proteção rigorosa contra Tip Stall

    // Saída principal para o Core 1 (Pitch/Roll PIDs)
    float roll_cmd_deg;

    L1Guidance();

    // Executa a malha de navegação a 50Hz (Core 0)
// Coordenadas agora em Double (64 bits) para evitar truncamento sub-métrico
    void compute(double current_lat, double current_lon, float ground_speed, float current_course,
                 double wp_prev_lat, double wp_prev_lon, double wp_next_lat, double wp_next_lon);

private:
    const float GRAVITY = 9.81f;
    const double EARTH_RADIUS = 6371000.0; // Double

    double getDistance(double lat1, double lon1, double lat2, double lon2);
    double getBearing(double lat1, double lon1, double lat2, double lon2);
};

#endif