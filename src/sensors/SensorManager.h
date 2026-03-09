#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

#define I2C_FAST_SDA 21
#define I2C_FAST_SCL 22
#define I2C_SLOW_SDA 32
#define I2C_SLOW_SCL 33

struct RawIMU {
    float ax, ay, az; 
    float gx, gy, gz; 
};

// Estrutura para os coeficientes de calibração únicos do BMP280
struct BMP280_Calib {
    uint16_t dig_T1; int16_t  dig_T2; int16_t  dig_T3;
    uint16_t dig_P1; int16_t  dig_P2; int16_t  dig_P3;
    int16_t  dig_P4; int16_t  dig_P5; int16_t  dig_P6;
    int16_t  dig_P7; int16_t  dig_P8; int16_t  dig_P9;
};

class SensorManager {
public:
    static void initSensors();
    static bool readIMU(RawIMU &imuData);
    
    // Retorna 'true' apenas se a leitura for matematicamente válida
    static bool readBaro(float &pressure, float &temperature);
    
    static void recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin);

private:
    static float gyroBias[3];
    static BMP280_Calib bmp_calib; // Armazena a calibração
    static int32_t t_fine;         // Variável global da Bosch para precisão térmica

    static void initMPU6050();
    static void initBMP280();
    static void readBMP280Calibration();
};

#endif