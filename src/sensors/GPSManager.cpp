#include "GPSManager.h"

// Instância estática do objeto TinyGPS++
TinyGPSPlus GPSManager::gps;

void GPSManager::init() {
    // Inicializa a Hardware Serial 2 do ESP32. 
    // Isto é vital: NÃO use SoftwareSerial no ESP32, consome demasiada CPU!
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS NEO-6M Inicializado na Serial2 (9600 baud).");
}

bool GPSManager::update() {
    bool newData = false;
    
    // Lê todos os bytes que estão na fila de receção da UART e alimenta o parser
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) {
            // Se a localização tiver sido atualizada neste exato pacote, levantamos a flag
            if (gps.location.isUpdated()) {
                newData = true;
            }
        }
    }
    return newData;
}

int32_t GPSManager::getLatitudeE7() {
    // Converte float puro (ex: -12.9714000) para um número inteiro longo (-129714000)
    // Isto evita problemas ao empacotar os 4 bytes para o LoRa
    return (int32_t)(gps.location.lat() * 10000000.0);
}

int32_t GPSManager::getLongitudeE7() {
    return (int32_t)(gps.location.lng() * 10000000.0);
}

float GPSManager::getSpeedMPS() {
    // A biblioteca devolve nativamente metros por segundo
    return gps.speed.mps();
}

float GPSManager::getCourseDeg() {
    // COG (Course Over Ground) - Fundamental para o algoritmo "Heading Híbrido"
    return gps.course.deg();
}

bool GPSManager::hasValidFix() {
    // Considera o Fix válido APENAS se o módulo reportar localização válida
    // E se o último pacote não for demasiado antigo (menos de 2000 ms).
    // Se a asa entrar num túnel e perder o sinal, isUpdated() para, e age() aumenta.
    return (gps.location.isValid() && gps.location.age() < 2000);
}