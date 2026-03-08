#include "SensorManager.h"

// Endereços I2C
#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

// Fatores de conversão (Para 8G e 2000DPS)
#define ACCEL_SCALE 4096.0f
#define GYRO_SCALE  16.4f

float SensorManager::gyroBias[3] = {0, 0, 0};

void SensorManager::initSensors() {
    // 1. Inicializa o I2C Rápido (Core 1 -> MPU6050)
    Wire.begin(I2C_FAST_SDA, I2C_FAST_SCL, 400000);
    Wire.setTimeOut(2); // Timeout de 2ms. Se o MPU não responder, aborta e não trava o Core 1!

    // 2. Inicializa o I2C Lento (Core 0 -> BMP280)
    Wire1.begin(I2C_SLOW_SDA, I2C_SLOW_SCL, 400000);
    Wire1.setTimeOut(5);

    initMPU6050();
    initBMP280();
}

void SensorManager::initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // Power Management 1
    Wire.write(0x00); // Acorda o sensor
    if (Wire.endTransmission() != 0) {
        Serial.println("ERRO: MPU6050 não detectado! Tentando Hard Reset...");
        recoverI2CBus(I2C_FAST_SDA, I2C_FAST_SCL);
        Wire.begin(I2C_FAST_SDA, I2C_FAST_SCL, 400000);
    }

    // Configura Filtro Passa-Baixa Digital (DLPF) para ~42Hz
    // EXTREMAMENTE NECESSÁRIO para limpar vibração da hélice (propwash)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); // CONFIG
    Wire.write(0x03); 
    Wire.endTransmission();

    // Configura Giroscópio para +/- 2000 graus/seg (Dinâmica de Asa Voadora)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x18); 
    Wire.endTransmission();

    // Configura Acelerômetro para +/- 8g (Evita saturação em curvas L1 violentas)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(0x10); 
    Wire.endTransmission();
}

void SensorManager::initBMP280() {
    // Configurações do BMP280 no Barramento 1 (Wire1)
    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0xF4); // ctrl_meas
    // Oversampling Temp x1, Pressure x4, Modo Normal
    Wire1.write(0x2F); 
    Wire1.endTransmission();

    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0xF5); // config
    // Standby 0.5ms, IIR Filter x16 (Essencial para o Filtro de Kalman Vertical!)
    Wire1.write(0x10); 
    Wire1.endTransmission();
}

// Leitura otimizada em "Burst" (14 bytes de uma vez) para o Core 1
bool SensorManager::readIMU(RawIMU &imuData) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Registrador inicial do Acelerômetro
    if (Wire.endTransmission(false) != 0) return false; // Falhou, não trava

    uint8_t bytesRead = Wire.requestFrom(MPU6050_ADDR, 14, true);
    if (bytesRead != 14) return false; // Erro de I2C, descarta leitura

    // Reconstrução rápida por bitshift
    int16_t ax = (Wire.read() << 8 | Wire.read());
    int16_t ay = (Wire.read() << 8 | Wire.read());
    int16_t az = (Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read(); // Ignora os 2 bytes de temperatura interna
    int16_t gx = (Wire.read() << 8 | Wire.read());
    int16_t gy = (Wire.read() << 8 | Wire.read());
    int16_t gz = (Wire.read() << 8 | Wire.read());

    // Conversão para escalas físicas reais
    imuData.ax = (float)ax / ACCEL_SCALE;
    imuData.ay = (float)ay / ACCEL_SCALE;
    imuData.az = (float)az / ACCEL_SCALE;
    imuData.gx = (float)gx / GYRO_SCALE - gyroBias[0];
    imuData.gy = (float)gy / GYRO_SCALE - gyroBias[1];
    imuData.gz = (float)gz / GYRO_SCALE - gyroBias[2];

    return true;
}

// =================================================================
// PROTOCOLO DE FAILSAFE DE HARDWARE (Conforme especificado no manual)
// Purga a capacitância parasita e "desengasga" escravos I2C teimosos
// =================================================================
void SensorManager::recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin) {
    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, INPUT_PULLUP);
    delay(10);
    
    pinMode(scl_pin, OUTPUT);
    // Pulsa o clock 9 vezes para forçar o escravo a liberar o SDA
    for (int i = 0; i < 9; i++) {
        digitalWrite(scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_pin, HIGH);
        delayMicroseconds(5);
    }
    
    // Gera condição de STOP física
    pinMode(sda_pin, OUTPUT);
    digitalWrite(sda_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda_pin, HIGH);
    delay(10);
    
    Serial.println("I2C Hard Reset executado.");
}