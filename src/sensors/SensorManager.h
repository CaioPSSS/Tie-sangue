#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

// Definição dos Barramentos Físicos Isolados
#define I2C_FAST_SDA 21
#define I2C_FAST_SCL 22
#define I2C_SLOW_SDA 32
#define I2C_SLOW_SCL 33

struct RawIMU {
    float ax, ay, az; // Força G
    float gx, gy, gz; // Graus por segundo (dps)
};

class SensorManager {
public:
    static void initSensors();
    static bool readIMU(RawIMU &imuData);
    static bool readBaro(float &pressure, float &temperature);
    
    // Failsafe Crítico: Destrava o I2C se o MPU6050 congelar
    static void recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin);

private:
    static float gyroBias[3];
    static void initMPU6050();
    static void initBMP280();
};

#endif