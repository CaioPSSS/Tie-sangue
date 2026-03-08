#include "AHRS.h"
#include <math.h>

AHRS::AHRS() {}

void AHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Converte giroscópio de graus/s para radianos/s
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // Calcula a magnitude da aceleração (Força G total)
    float acc_mag = sqrt(ax * ax + ay * ay + az * az);

    // LÓGICA DE PREVENÇÃO DE CURVA CENTRÍFUGA (Anti-Centrifugal Clamp)
    // Se a aceleração fugir muito de 1G (ex: durante curvas bancadas fechadas),
    // reduzimos a confiança no acelerômetro para que ele não entorte o horizonte.
    float dynamic_Kp = Kp;
    if(acc_mag > 0.1f) {
        float g_error = abs(acc_mag - 1.0f);
        // Se erro > 0.2G, zera o Kp (só confia no giroscópio momentaneamente)
        if(g_error > 0.2f) dynamic_Kp = 0.0f; 
        else dynamic_Kp *= (1.0f - (g_error * 5.0f)); // Redução linear suave
    }

    // Só usa o acelerômetro se a leitura for válida e não for queda livre total
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normaliza o vetor do acelerômetro
        recipNorm = 1.0f / acc_mag;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estima a direção da gravidade com base no Quatérnio atual
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Erro: Produto vetorial entre gravidade estimada e aceleração medida
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Aplica o Ganho Integral (Ki) para corrigir o Bias do giroscópio ao longo do tempo
        if(Ki > 0.0f) {
            integralFBx += Ki * halfex * dt;
            integralFBy += Ki * halfey * dt;
            integralFBz += Ki * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        // Aplica o Ganho Proporcional (Kp Dinâmico)
        gx += dynamic_Kp * halfex;
        gy += dynamic_Kp * halfey;
        gz += dynamic_Kp * halfez;
    }

    // Integra a taxa do giroscópio para atualizar o Quatérnio
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normaliza o novo Quatérnio
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Atualiza as saídas públicas da classe
    computeEulerAngles();
    
    // Só recalculamos a aceleração inercial se a leitura do acelerômetro foi válida
    if(acc_mag > 0.1f) {
        computeEarthZAccel(ax * acc_mag, ay * acc_mag, az * acc_mag); // Envia os Gs originais
    }
}

void AHRS::computeEulerAngles() {
    // Conversão de Quatérnio para Euler (Roll, Pitch, Yaw)
    // A ordem de rotação obedece o padrão aeroespacial (Tait-Bryan)
    roll  = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / PI);
    pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0f / PI);
    yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / PI);
}

void AHRS::computeEarthZAccel(float ax, float ay, float az) {
    // Matriz de Rotação Inversa para descobrir a componente Z pura da Terra
    // Essa equação anula o efeito da inclinação do avião sobre o sensor.
    earth_z_accel = 2.0f * (q1 * q3 - q0 * q2) * ax + 
                    2.0f * (q2 * q3 + q0 * q1) * ay + 
                    (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;
                    
    // Se o avião estiver parado na mesa, earth_z_accel será ~1.0G.
    // Para o variômetro, no Core 0, você fará (earth_z_accel - 1.0f) * 9.81 para ter m/s².
}