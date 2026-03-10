#include "SensorManager.h"
#include "driver/i2c.h" // Acesso direto ao núcleo do ESP-IDF!

// Endereços I2C
#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

// Fatores de conversão (Para 8G e 2000DPS)
#define ACCEL_SCALE 4096.0f
#define GYRO_SCALE  16.4f

// Vincula o MPU6050 à porta I2C número 0 do ESP32 (Acelerador de Hardware)
#define I2C_MASTER_NUM I2C_NUM_0 

float SensorManager::gyroBias[3] = {0, 0, 0};
float SensorManager::ground_pressure_pa = 101325.0f; // Nível do mar por defeito
BMP280_Calib SensorManager::bmp_calib;
int32_t SensorManager::t_fine;

NotchFilter SensorManager::notch_gx;
NotchFilter SensorManager::notch_gy;
NotchFilter SensorManager::notch_gz;

void SensorManager::initSensors() {
    // 1. INICIALIZA O I2C0 NATIVO DO ESP-IDF (Core 1 -> MPU6050) - Não bloqueante!
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_FAST_SDA;
    conf.scl_io_num = I2C_FAST_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000; // 400 kHz Fast Mode
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // 2. Inicializa o I2C Lento padrão (Core 0 -> BMP280)
    Wire1.begin(I2C_SLOW_SDA, I2C_SLOW_SCL, 400000);
    Wire1.setTimeOut(5);

    initMPU6050();
    initBMP280();
}

// ==========================================
// CALIBRAÇÃO DO GIROSCÓPIO (FIM DO DRIFT)
// ==========================================
void SensorManager::calibrateIMU() {
    Serial.println("Calibrando IMU... Mantenha a asa estatica!");
    
    // Zera os bias anteriores
    gyroBias[0] = 0; gyroBias[1] = 0; gyroBias[2] = 0;
    
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 500;
    RawIMU tempIMU;

    delay(500); // Dá tempo para a mão do piloto largar a asa

    for (int i = 0; i < samples; i++) {
        readIMU(tempIMU); 
        sum_gx += tempIMU.gx;
        sum_gy += tempIMU.gy;
        sum_gz += tempIMU.gz;
        delay(2); 
    }

    // Calcula a média do erro estático
    gyroBias[0] = sum_gx / samples;
    gyroBias[1] = sum_gy / samples;
    gyroBias[2] = sum_gz / samples;

    Serial.printf("IMU Calibrada! Erros anulados -> X:%.2f, Y:%.2f, Z:%.2f\n", 
                  gyroBias[0], gyroBias[1], gyroBias[2]);
}

// ==========================================
// CALIBRAÇÃO DO BARÓMETRO (ALTITUDE AGL ZERO)
// ==========================================
void SensorManager::calibrateBaro() {
    Serial.println("Calibrando Barometro (Zero Ground)...");
    
    float sum_p = 0;
    float temp_p, temp_t;
    int valid_samples = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (readBaro(temp_p, temp_t)) {
            sum_p += temp_p;
            valid_samples++;
        }
        delay(20); 
    }

    if (valid_samples > 0) {
        ground_pressure_pa = sum_p / valid_samples;
        Serial.printf("Barometro Calibrado! Pressao de Solo: %.2f Pa\n", ground_pressure_pa);
    } else {
        Serial.println("ERRO CRITICO: Falha na calibracao do Barometro.");
    }
}

// ==========================================
// FÓRMULA BAROMÉTRICA (ALTITUDE RELATIVA)
// ==========================================
float SensorManager::getAltitudeAGL(float current_pressure_pa) {
    if (current_pressure_pa <= 0) return 0.0f;
    return 44330.0f * (1.0f - pow(current_pressure_pa / ground_pressure_pa, 0.190295f));
}

// ==========================================
// INICIALIZAÇÃO E HARDWARE ESP-IDF (MPU6050)
// ==========================================
void SensorManager::initMPU6050() {
    // Comando para Acordar o Sensor usando ESP-IDF
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // Power Management
    i2c_master_write_byte(cmd, 0x00, true); // Acorda
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    // Configurações de Filtro de Hardware (DLPF 20Hz - Blindagem contra propwash)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1A, true); 
    i2c_master_write_byte(cmd, 0x04, true); // 0x04 = 20Hz
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    // Configuração Giroscópio (+/- 2000 dps)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true); 
    i2c_master_write_byte(cmd, 0x18, true); 
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    // Configuração Acelerómetro (+/- 8g)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1C, true); 
    i2c_master_write_byte(cmd, 0x10, true); 
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    // Inicializa Filtros Notch de Software (Centro 150Hz, Largura 50Hz, dt 0.004s)
    notch_gx.init(150.0f, 50.0f, 0.004f);
    notch_gy.init(150.0f, 50.0f, 0.004f);
    notch_gz.init(150.0f, 50.0f, 0.004f);
}

void SensorManager::initBMP280() {
    readBMP280Calibration();

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

// =================================================================
// LEITURA DE ALTA VELOCIDADE E HARDWARE TIMEOUT (ESP-IDF)
// =================================================================
bool SensorManager::readIMU(RawIMU &imuData) {
    uint8_t buffer[14];
    
    // Constrói a sequência de comandos para o Controlador DMA do ESP32
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // Pede para ler a partir do ACCEL_XOUT_H
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 14, I2C_MASTER_LAST_NACK); // Lê 14 bytes de rajada
    i2c_master_stop(cmd);

    // Dispara o hardware! Timeout estrito de 2 TICKS (2 ms).
    // Se o MPU6050 não responder, o ESP32 aborta e o Core 1 não trava.
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(2));
    i2c_cmd_link_delete(cmd);

    // Se falhar em ler, devolve false (Task usa os dados antigos/inerciais)
    if (ret != ESP_OK) return false;

    // Reconstrói os inteiros
    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];
    int16_t gx = (buffer[8] << 8) | buffer[9];
    int16_t gy = (buffer[10] << 8) | buffer[11];
    int16_t gz = (buffer[12] << 8) | buffer[13];

    // Escala e Filtra o Bias
    imuData.ax = (float)ax / ACCEL_SCALE;
    imuData.ay = (float)ay / ACCEL_SCALE;
    imuData.az = (float)az / ACCEL_SCALE;
    
    imuData.gx = ((float)gx / GYRO_SCALE) - gyroBias[0];
    imuData.gy = ((float)gy / GYRO_SCALE) - gyroBias[1];
    imuData.gz = ((float)gz / GYRO_SCALE) - gyroBias[2];

    // A MÁGICA ACONTECE AQUI: Aplica o Filtro Notch para exterminar o ruído harmónico
    imuData.gx = notch_gx.apply(imuData.gx);
    imuData.gy = notch_gy.apply(imuData.gy);
    imuData.gz = notch_gz.apply(imuData.gz);

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

    // ERROR CHECK 3: Sensor morto
    if (adc_P == 0 || adc_P == 0xFFFFF) return false;

    // --- MATEMÁTICA OFICIAL DA BOSCH ---
    int32_t var1_t = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2_t = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;
    t_fine = var1_t + var2_t;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f; // Celsius

    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)bmp_calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)bmp_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1_p * (int64_t)bmp_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bmp_calib.dig_P1) >> 33;

    if (var1_p == 0) return false;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    
    pressure = (float)p / 256.0f; // Pascals (Pa)

    // ERROR CHECK 4 (Sanity Bounds)
    if (pressure < 60000.0f || pressure > 115000.0f) return false;

    return true; 
}

// =================================================================
// PROTOCOLO DE FAILSAFE DE HARDWARE
// =================================================================
void SensorManager::recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin) {
    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, INPUT_PULLUP);
    delay(10);
    
    pinMode(scl_pin, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_pin, HIGH);
        delayMicroseconds(5);
    }
    
    pinMode(sda_pin, OUTPUT);
    digitalWrite(sda_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda_pin, HIGH);
    delay(10);
    
    Serial.println("I2C Hard Reset executado.");
}