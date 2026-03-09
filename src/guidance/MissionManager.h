#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include <Arduino.h>

// Estrutura de um Waypoint (Marcador Cartográfico Tridimensional)
struct Waypoint {
    float lat;
    float lon;
    float altitude_m;
    float speed_ms;
};

// Estados Comportamentais do RTH (Conforme documentação do projeto)
enum RTH_State {
    RTH_IDLE = 0,     // RTH Desligado
    RTH_CLIMB,        // Ganho de Altitude Segura (Climb-First)
    RTH_RETURN,       // Virada Vetorial e Cruzeiro para Casa
    RTH_LOITER_DOWN   // Espiral Concêntrica e Regulação (Pouso)
};

class MissionManager {
public:
    // Parâmetros de Navegação e Segurança (Tuning)
    float hit_radius_m = 30.0f;           // Distância para considerar o WP alcançado
    float nav_rth_altitude = 80.0f;       // Altitude de segurança para voltar
    float rtl_descend_alt = 20.0f;        // Altitude final da órbita de pouso
    float cruise_speed = 15.0f;           // Velocidade econômica do TECS

    MissionManager();

    // 1. Controle da Missão
    void loadMission(); // (Mockup) Carrega WPs para a RAM
    void update(float current_lat, float current_lon, float current_alt, bool is_rth_active);
    void saveWaypoint(uint8_t index, float lat, float lon, float alt, float speed);

    // 2. Definir o Ponto de Retorno (Home)
    void setHome(float lat, float lon, float alt);

    // 3. Getters para o L1 Guidance e TECS
    float getActiveLat();
    float getActiveLon();
    float getPrevLat();
    float getPrevLon();
    float getActiveAltitude();
    float getActiveSpeed();

private:
    const float EARTH_RADIUS = 6371000.0f;

    // RAM Matrix para Waypoints (Máximo de 20 para poupar memória do ESP32)
    static const int MAX_WAYPOINTS = 20;
    Waypoint mission[MAX_WAYPOINTS];
    
    int total_waypoints;
    int current_wp_index;

    // Coordenadas de Casa (Home)
    Waypoint home_wp;

    // Variáveis da Máquina de Estados do RTH
    RTH_State rth_state;
    Waypoint rth_climb_point; // Ponto virtual para subir em espiral antes de voltar

    // Matemática Geodésica (Haversine)
    float getDistance(float lat1, float lon1, float lat2, float lon2);
};

#endif