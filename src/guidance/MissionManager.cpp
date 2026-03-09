#include "MissionManager.h"
#include <math.h>

MissionManager::MissionManager() {
    total_waypoints = 0;
    current_wp_index = 0;
    rth_state = RTH_IDLE;
    has_valid_home = false; // Começa sempre como falso por segurança
    
    home_wp = {0.0f, 0.0f, 0.0f, cruise_speed};
    loadMission();
}

void MissionManager::setHome(float lat, float lon, float alt) {
    home_wp.lat = lat;
    home_wp.lon = lon;
    home_wp.altitude_m = alt;
    has_valid_home = true; // <--- HOME TRAVADO EM SEGURANÇA!
    Serial.printf("HOME POINT REGISTRADO: Lat: %.6f, Lon: %.6f, Alt: %.1f\n", lat, lon, alt);
}

void MissionManager::loadMission() {
    // Inicialização vazia (Os Waypoints chegam agora via LoRa!)
    total_waypoints = 0;
    current_wp_index = 0;
}

void MissionManager::saveWaypoint(uint8_t index, float lat, float lon, float alt, float speed) {
    if (index < MAX_WAYPOINTS) {
        mission[index].lat = lat;
        mission[index].lon = lon;
        mission[index].altitude_m = alt;
        mission[index].speed_ms = speed;

        if (index >= total_waypoints) {
            total_waypoints = index + 1;
        }
        Serial.printf("WP %d Gravado na RAM! Lat: %.6f, Lon: %.6f\n", index, lat, lon);
    }
}

void MissionManager::update(float current_lat, float current_lon, float current_alt, bool is_rth_active) {
    
    // ==========================================================
    // MÁQUINA DE ESTADOS: RETURN TO HOME (Emergência / Failsafe)
    // ==========================================================
    if (is_rth_active) {
        
        if (rth_state == RTH_IDLE) {
            Serial.println("RTH ENGATILHADO! Iniciando Climb-First Protocol.");
            rth_state = RTH_CLIMB;
            
            // Grava a coordenada exata de onde o sinal caiu para o caso de não haver Home.
            rth_climb_point.lat = current_lat;
            rth_climb_point.lon = current_lon;
        }

        switch (rth_state) {
            case RTH_CLIMB:
                if (current_alt >= nav_rth_altitude - 2.0f) {
                    // BLINDAGEM CONTRA O NULL ISLAND BUG
                    if (has_valid_home) {
                        Serial.println("Altitude RTH atingida. Virando para Casa.");
                        rth_state = RTH_RETURN;
                    } else {
                        Serial.println("ALERTA CRITICO: Sem Home Point! Iniciando descida em espiral no local (Loiter Down).");
                        rth_state = RTH_LOITER_DOWN;
                    }
                }
                break;

            case RTH_RETURN:
                if (getDistance(current_lat, current_lon, home_wp.lat, home_wp.lon) < 50.0f) {
                    Serial.println("Chegou a Casa! Iniciando Espiral Descendente.");
                    rth_state = RTH_LOITER_DOWN;
                }
                break;

            case RTH_LOITER_DOWN:
                // O TECS trata do afundamento.
                break;
                
            default:
                break;
        }
        return; 
    } 
    
    if (rth_state != RTH_IDLE) {
        rth_state = RTH_IDLE;
        Serial.println("RTH Abortado. Piloto retomou o controle.");
    }

    // ==========================================================
    // GESTÃO DE MISSÃO NORMAL
    // ==========================================================
    if (current_wp_index < total_waypoints && total_waypoints > 0) {
        float dist_to_wp = getDistance(current_lat, current_lon, 
                                       mission[current_wp_index].lat, 
                                       mission[current_wp_index].lon);

        if (dist_to_wp <= hit_radius_m) {
            Serial.printf("Waypoint %d atingido!\n", current_wp_index);
            current_wp_index++; 
            
            if (current_wp_index >= total_waypoints) {
                Serial.println("Missao Concluida! Entrando em RTH Automatico.");
                // Modifica globalState ou força a variável internamente no futuro
            }
        }
    }
}

// ==========================================================
// GETTERS COM BLINDAGEM DE FALHAS
// ==========================================================
float MissionManager::getActiveLat() {
    if (rth_state == RTH_CLIMB) return rth_climb_point.lat;
    if (rth_state == RTH_RETURN) return home_wp.lat;
    if (rth_state == RTH_LOITER_DOWN) {
        // Se não tiver casa, orbita no ponto onde o sinal caiu!
        return has_valid_home ? home_wp.lat : rth_climb_point.lat; 
    }
    
    if (current_wp_index < total_waypoints && total_waypoints > 0) return mission[current_wp_index].lat;
    return rth_climb_point.lat; // Fallback de segurança
}

float MissionManager::getActiveLon() {
    if (rth_state == RTH_CLIMB) return rth_climb_point.lon;
    if (rth_state == RTH_RETURN) return home_wp.lon;
    if (rth_state == RTH_LOITER_DOWN) {
        return has_valid_home ? home_wp.lon : rth_climb_point.lon; 
    }
    
    if (current_wp_index < total_waypoints && total_waypoints > 0) return mission[current_wp_index].lon;
    return rth_climb_point.lon;
}

float MissionManager::getPrevLat() {
    if (rth_state != RTH_IDLE) return rth_climb_point.lat; 
    
    if (current_wp_index == 0) return has_valid_home ? home_wp.lat : rth_climb_point.lat; 
    if (current_wp_index < total_waypoints) return mission[current_wp_index - 1].lat;
    return rth_climb_point.lat;
}

float MissionManager::getPrevLon() {
    if (rth_state != RTH_IDLE) return rth_climb_point.lon;
    
    if (current_wp_index == 0) return has_valid_home ? home_wp.lon : rth_climb_point.lon;
    if (current_wp_index < total_waypoints) return mission[current_wp_index - 1].lon;
    return rth_climb_point.lon;
}

float MissionManager::getActiveAltitude() {
    if (rth_state == RTH_CLIMB || rth_state == RTH_RETURN) return nav_rth_altitude; 
    if (rth_state == RTH_LOITER_DOWN) return rtl_descend_alt;                       
    
    if (current_wp_index < total_waypoints && total_waypoints > 0) return mission[current_wp_index].altitude_m;
    return rtl_descend_alt;
}

float MissionManager::getActiveSpeed() {
    if (rth_state != RTH_IDLE) return cruise_speed;
    
    if (current_wp_index < total_waypoints && total_waypoints > 0) return mission[current_wp_index].speed_ms;
    return cruise_speed;
}

float MissionManager::getDistance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * (PI / 180.0f);
    float dLon = (lon2 - lon1) * (PI / 180.0f);
    lat1 = lat1 * (PI / 180.0f);
    lat2 = lat2 * (PI / 180.0f);

    float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
              sin(dLon / 2.0f) * sin(dLon / 2.0f) * cos(lat1) * cos(lat2);
    float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
    
    return EARTH_RADIUS * c; 
}