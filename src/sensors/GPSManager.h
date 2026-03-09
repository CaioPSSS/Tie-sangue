#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPS++.h> // Instale a biblioteca "TinyGPSPlus" de Mikal Hart

// --------------------------------------------------------
// MAPEAMENTO DE PINOS (Hardware UART 2 do ESP32)
// --------------------------------------------------------
#define GPS_RX_PIN 16 // Liga ao pino TX do NEO-6M
#define GPS_TX_PIN 17 // Liga ao pino RX do NEO-6M
#define GPS_BAUD   9600

class GPSManager {
public:
    static void init();

    // Lê o buffer da porta série. Retorna 'true' se uma nova localização for validada.
    static bool update();

    // Getters para extrair os dados processados para a memória partilhada (IPC)
    static int32_t getLatitudeE7();
    static int32_t getLongitudeE7();
    static float getSpeedMPS();
    static float getCourseDeg();
    static bool hasValidFix();

private:
    static TinyGPSPlus gps;
};

#endif