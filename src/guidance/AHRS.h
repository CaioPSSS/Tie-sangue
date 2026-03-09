#ifndef AHRS_H
#define AHRS_H

#include <Arduino.h>

class AHRS {
public:
    // Ganhos do Filtro Complementar (Mahony)
    float Kp = 2.0f;  // Ganho Proporcional: Confiança no Acelerômetro
    float Ki = 0.05f; // Ganho Integral: Corrige a deriva (drift) lenta do Giroscópio

    // Saídas (Prontas para o Controlador PID e para o Core 0)
    float roll, pitch, yaw;       // Ângulos em graus
    float earth_z_accel;          // Aceleração vertical real (1G = parado)

    AHRS();

    // Atualiza a matemática. Deve ser chamado no Core 1 a ~250Hz.
    // gx, gy, gz em graus/s | ax, ay, az em Gs | dt em segundos
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt, float cog_error_deg = 0.0f);

private:
    // Estado do Quatérnio (Inicializado apontando para frente e nivelado)
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    
    // Acumulador do erro integral
    float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

    // Converte Quatérnio para Roll, Pitch, Yaw
    void computeEulerAngles();
    
    // Calcula a Aceleração Z no referencial inercial (Terra)
    void computeEarthZAccel(float ax, float ay, float az);
};

#endif