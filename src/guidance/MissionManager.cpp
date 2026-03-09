#include "MissionManager.h"
#include <math.h>

MissionManager::MissionManager() {
    total_waypoints = 0;
    current_wp_index = 0;
    rth_state = RTH_IDLE;
    
    // Zera o Home
    home_wp = {0.0f, 0.0f, 0.0f, cruise_speed};
    
    // Carrega a missão inicial
    loadMission();
}

void MissionManager::setHome(float lat, float lon, float alt) {
    home_wp.lat = lat;
    home_wp.lon = lon;
    home_wp.altitude_m = alt;
    Serial.printf("HOME POINT REGISTRADO: Lat: %.6f, Lon: %.6f, Alt: %.1f\n", lat, lon, alt);
}

void MissionManager::loadMission() {
    // Exemplo: Popula a matriz RAM com Waypoints reais em Salvador (Mockup para testes)
    mission[0] = {-12.971000f, -38.501000f, 50.0f, 15.0f}; // WP 1
    mission[1] = {-12.972000f, -38.502000f, 60.0f, 15.0f}; // WP 2
    mission[2] = {-12.973000f, -38.501000f, 50.0f, 18.0f}; // WP 3
    total_waypoints = 3;
    current_wp_index = 0;
}

void MissionManager::update(float current_lat, float current_lon, float current_alt, bool is_rth_active) {
    
    // ==========================================================
    // MÁQUINA DE ESTADOS: RETURN TO HOME (Emergência / Failsafe)
    // ==========================================================
    if (is_rth_active) {
        
        // Gatilho Inicial: Se acabou de entrar em RTH, tranca o estado em CLIMB
        if (rth_state == RTH_IDLE) {
            Serial.println("RTH ENGATILHADO! Iniciando Climb-First Protocol.");
            rth_state = RTH_CLIMB;
            
            // Grava a coordenada atual como um ponto de "Loiter" (Órbita) temporário 
            // para o avião subir em espiral antes de apontar para casa.
            rth_climb_point.lat = current_lat;
            rth_climb_point.lon = current_lon;
        }

        float dist_to_home = getDistance(current_lat, current_lon, home_wp.lat, home_wp.lon);

        switch (rth_state) {
            case RTH_CLIMB:
                // Sobe em espiral no local atual até atingir a altitude de segurança (nav_rth_altitude)
                if (current_alt >= nav_rth_altitude - 2.0f) {
                    Serial.println("Altitude de RTH atingida. Virando para Casa (Turn & Cruise).");
                    rth_state = RTH_RETURN;
                }
                break;

            case RTH_RETURN:
                // Navega em linha reta para casa. Se chegar perto (ex: raio de 50 metros), inicia pouso.
                if (dist_to_home < 50.0f) {
                    Serial.println("Chegou em Casa! Iniciando Espiral Descendente (Loiter Down).");
                    rth_state = RTH_LOITER_DOWN;
                }
                break;

            case RTH_LOITER_DOWN:
                // Fica a orbitar em cima do Home Point e pede ao TECS para afundar a altitude.
                // A altitude alvo já está a ser tratada pelo getActiveAltitude()
                break;
                
            default:
                break;
        }
        return; // Interrompe o processamento da missão normal se o RTH estiver ativo
    } 
    
    // Se o RTH foi desativado (Piloto retomou o sinal), reseta a FSM de emergência
    if (rth_state != RTH_IDLE) {
        rth_state = RTH_IDLE;
        Serial.println("RTH Abortado. Retomando Missao Automatica.");
    }

    // ==========================================================
    // GESTÃO DE MISSÃO NORMAL (Waypoints)
    // ==========================================================
    if (current_wp_index < total_waypoints) {
        // Calcula a distância usando Haversine
        float dist_to_wp = getDistance(current_lat, current_lon, 
                                       mission[current_wp_index].lat, 
                                       mission[current_wp_index].lon);

        // Se o VANT entrou na "Hit Sphere" (Raio de aceitação do Waypoint)
        if (dist_to_wp <= hit_radius_m) {
            Serial.printf("Waypoint %d atingido!\n", current_wp_index);
            current_wp_index++; // Avança para o próximo
            
            if (current_wp_index >= total_waypoints) {
                Serial.println("Missao Concluida! Entrando em Loiter no ultimo ponto.");
                // Aqui você poderia forçar um modo RTH automático no final da missão
            }
        }
    }
}

// ==========================================================
// GETTERS PARA ALIMENTAR O L1 GUIDANCE E O TECS
// ==========================================================

float MissionManager::getActiveLat() {
    if (rth_state == RTH_CLIMB) return rth_climb_point.lat;
    if (rth_state == RTH_RETURN || rth_state == RTH_LOITER_DOWN) return home_wp.lat;
    
    // Missão Normal
    if (current_wp_index < total_waypoints) return mission[current_wp_index].lat;
    return mission[total_waypoints - 1].lat; // Missão acabou, fica no último WP
}

float MissionManager::getActiveLon() {
    if (rth_state == RTH_CLIMB) return rth_climb_point.lon;
    if (rth_state == RTH_RETURN || rth_state == RTH_LOITER_DOWN) return home_wp.lon;
    
    // Missão Normal
    if (current_wp_index < total_waypoints) return mission[current_wp_index].lon;
    return mission[total_waypoints - 1].lon;
}

// O Prev WP é fundamental para o L1 Guidance calcular a linha reta (Track)
float MissionManager::getPrevLat() {
    if (rth_state != RTH_IDLE) return rth_climb_point.lat; // A linha é do ponto que iniciou o RTH até Casa
    
    if (current_wp_index == 0) return home_wp.lat; // Se está a ir para o 1º WP, a reta vem de Casa
    if (current_wp_index < total_waypoints) return mission[current_wp_index - 1].lat;
    return mission[total_waypoints - 1].lat;
}

float MissionManager::getPrevLon() {
    if (rth_state != RTH_IDLE) return rth_climb_point.lon;
    
    if (current_wp_index == 0) return home_wp.lon;
    if (current_wp_index < total_waypoints) return mission[current_wp_index - 1].lon;
    return mission[total_waypoints - 1].lon;
}

float MissionManager::getActiveAltitude() {
    if (rth_state == RTH_CLIMB || rth_state == RTH_RETURN) return nav_rth_altitude; // Ex: Sobe para 80m
    if (rth_state == RTH_LOITER_DOWN) return rtl_descend_alt;                       // Ex: Afunda para 20m
    
    // Missão Normal
    if (current_wp_index < total_waypoints) return mission[current_wp_index].altitude_m;
    return mission[total_waypoints - 1].altitude_m;
}

float MissionManager::getActiveSpeed() {
    // RTH usa velocidade econômica máxima (Cruise Speed) para salvar bateria
    if (rth_state != RTH_IDLE) return cruise_speed;
    
    // Missão Normal pode ter velocidades agressivas (ex: corrida)
    if (current_wp_index < total_waypoints) return mission[current_wp_index].speed_ms;
    return mission[total_waypoints - 1].speed_ms;
}

// ==========================================================
// MATEMÁTICA GEODÉSICA (Fórmula Haversine)
// ==========================================================
float MissionManager::getDistance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * (PI / 180.0f);
    float dLon = (lon2 - lon1) * (PI / 180.0f);
    lat1 = lat1 * (PI / 180.0f);
    lat2 = lat2 * (PI / 180.0f);

    float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
              sin(dLon / 2.0f) * sin(dLon / 2.0f) * cos(lat1) * cos(lat2);
    float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
    
    return EARTH_RADIUS * c; // Distância puramente em metros
}