#include "SensorManager.h"

// Endereços I2C
#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

// Fatores de conversão (Para 8G e 2000DPS)
#define ACCEL_SCALE 4096.0f
#define GYRO_SCALE  16.4f

float SensorManager::gyroBias[3] = {0, 0, 0};
BMP280_Calib SensorManager::bmp_calib;
int32_t SensorManager::t_fine;

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
    // 1. Lê a ROM de calibração do chip primeiro
    readBMP280Calibration();

    // 2. Configurações de Filtro e Oversampling
    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0xF4); // ctrl_meas
    Wire1.write(0x2F); // Temp x1, Pressure x4, Normal mode
    Wire1.endTransmission();

    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0xF5); // config
    Wire1.write(0x10); // Standby 0.5ms, IIR Filter x16
    Wire1.endTransmission();
}

void SensorManager::readBMP280Calibration() {
    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0x88); // Início dos registradores de calibração
    Wire1.endTransmission();
    
    Wire1.requestFrom(BMP280_ADDR, 24);
    if(Wire1.available() == 24) {
        bmp_calib.dig_T1 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_T2 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_T3 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P1 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P2 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P3 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P4 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P5 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P6 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P7 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P8 = Wire1.read() | (Wire1.read() << 8);
        bmp_calib.dig_P9 = Wire1.read() | (Wire1.read() << 8);
    }
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

bool SensorManager::readBaro(float &pressure, float &temperature) {
    Wire1.beginTransmission(BMP280_ADDR);
    Wire1.write(0xF7); // Registrador inicial de dados (Pressão MSB)
    
    // ERROR CHECK 1: Dispositivo respondeu?
    if (Wire1.endTransmission(false) != 0) return false;

    uint8_t bytesRead = Wire1.requestFrom(BMP280_ADDR, 6, true);
    
    // ERROR CHECK 2: Leu a quantidade certa de bytes?
    if (bytesRead != 6) return false;

    int32_t adc_P = (Wire1.read() << 12) | (Wire1.read() << 4) | (Wire1.read() >> 4);
    int32_t adc_T = (Wire1.read() << 12) | (Wire1.read() << 4) | (Wire1.read() >> 4);

    // ERROR CHECK 3: Sensor morto / desconectado no barramento (Geralmente retorna 0x00000 ou 0xFFFFF)
    if (adc_P == 0 || adc_P == 0xFFFFF) return false;

    // --- MATEMÁTICA OFICIAL DE COMPENSAÇÃO DA BOSCH ---
    
    // Compensação de Temperatura (Necessária para a pressão)
    int32_t var1_t = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2_t = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;
    t_fine = var1_t + var2_t;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f; // Celsius

    // Compensação de Pressão
    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)bmp_calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)bmp_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1_p * (int64_t)bmp_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bmp_calib.dig_P1) >> 33;

    if (var1_p == 0) return false; // Evita divisão por zero matemática!

    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    
    pressure = (float)p / 256.0f; // Pascals (Pa)

    // ERROR CHECK 4 (Sanity Bounds):
    // Se a pressão estiver fora dos limites habitáveis da Terra, é ruído.
    if (pressure < 60000.0f || pressure > 115000.0f) return false;

    return true; // Leitura limpa, segura e calibrada!
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