#ifndef FLIGHT_MODE_MANAGER_H
#define FLIGHT_MODE_MANAGER_H

#include <Arduino.h>
#include "../common/SharedTypes.h"
#include "../control/PID.h"

class FlightModeManager {
public:
    // --- SAÍDAS DECIDIDAS PELA MÁQUINA DE ESTADOS ---
    float final_roll_target;    // Vai para o Angle PID
    float final_pitch_target;   // Vai para o Angle PID
    float final_throttle_pwm;   // Vai direto para o Motor
    
    // Se for verdadeiro, a Task_FlightLoop ignora os PIDs e manda o sinal direto para os servos
    bool bypass_pids; 

    FlightModeManager();

    // Executa a árvore de decisão a 250Hz no Core 1
    void update(uint8_t current_mode, 
                float rc_roll, float rc_pitch, float rc_throttle,       // Comandos Humanos
                float nav_roll, float nav_pitch, float nav_throttle,    // Comandos do Core 0 (L1/TECS)
                PID &rollAnglePID, PID &pitchAnglePID,                  // Referências para Reset
                PID &rollRatePID, PID &pitchRatePID);

private:
    uint8_t previous_mode;
};

#endif