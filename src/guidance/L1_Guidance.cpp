#include "L1_Guidance.h"
#include <math.h>

L1Guidance::L1Guidance() {
    roll_cmd_deg = 0.0f;
}

void L1Guidance::compute(double current_lat, double current_lon, float ground_speed, float current_course,
                         double wp_prev_lat, double wp_prev_lon, double wp_next_lat, double wp_next_lon) {
    
    // 1. Proteção: Se a velocidade for muito baixa (ex: no chão ou contra vento extremo), 
    // evitamos divisões por zero e mantemos a asa nivelada.
    if (ground_speed < 3.0f) {
        roll_cmd_deg = 0.0f;
        return;
    }

    // 2. Calcula a distância L1 dinâmica baseada na velocidade e período
    // Quanto mais rápido o avião, mais longe ele "olha" para frente para não ziguezaguear.
    float l1_distance = (1.0f / PI) * l1_damping * l1_period * ground_speed;
    
    // Limite mínimo de L1 para evitar agressividade extrema perto do Waypoint
    if (l1_distance < 10.0f) l1_distance = 10.0f;

    // 3. Descobre o rumo (Bearing) ideal da linha entre o WP anterior e o Próximo
    float track_bearing = getBearing(wp_prev_lat, wp_prev_lon, wp_next_lat, wp_next_lon);
    
    // 4. Descobre o rumo direto do avião para o próximo WP
    float direct_bearing = getBearing(current_lat, current_lon, wp_next_lat, wp_next_lon);

    // 5. Calcula o Cross-Track Error (O quanto estamos fora do "trilho" virtual)
    float distance_to_next_wp = getDistance(current_lat, current_lon, wp_next_lat, wp_next_lon);
    
    // Diferença angular entre a rota ideal e onde o avião está em relação ao alvo
    float track_error_angle = direct_bearing - track_bearing;
    
    // Normaliza ângulo para -180 a 180 (radianos)
    while (track_error_angle > PI) track_error_angle -= 2.0f * PI;
    while (track_error_angle < -PI) track_error_angle += 2.0f * PI;

    float cross_track_error = distance_to_next_wp * sin(track_error_angle);

    // 6. Calcula o ângulo Eta (η) do algoritmo L1
    // É o ângulo entre a velocidade atual e o ponto de visada L1.
    float eta = asin(constrain(cross_track_error / l1_distance, -1.0f, 1.0f));

    // 7. Aceleração Lateral Comandada (L1 Math)
    float lateral_accel = 2.0f * ((ground_speed * ground_speed) / l1_distance) * sin(eta);

    // 8. Converte a aceleração lateral em Comando de Rolagem (Bank Angle)
    // a = g * tan(phi) -> phi = atan(a / g)
    float desired_roll_rad = atan2(lateral_accel, GRAVITY);
    
    // Converte para graus
    float desired_roll_deg = desired_roll_rad * (180.0f / PI);

    // 9. Saturação (Clamp) de segurança
    // Nunca deixa a asa voadora virar de faca e cair de asa.
    roll_cmd_deg = constrain(desired_roll_deg, -max_roll_angle, max_roll_angle);
}

// ---------------------------------------------------------
// Funções Geodésicas Auxiliares (Usando Radianos)
// ---------------------------------------------------------
double L1Guidance::getDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * (M_PI / 180.0);
    double dLon = (lon2 - lon1) * (M_PI / 180.0);
    lat1 = lat1 * (M_PI / 180.0);
    lat2 = lat2 * (M_PI / 180.0);

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               sin(dLon / 2.0) * sin(dLon / 2.0) * cos(lat1) * cos(lat2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS * c;
}

double L1Guidance::getBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = lat1 * (M_PI / 180.0);
    lat2 = lat2 * (M_PI / 180.0);
    double dLon = (lon2 - lon1) * (M_PI / 180.0);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return atan2(y, x); 
}