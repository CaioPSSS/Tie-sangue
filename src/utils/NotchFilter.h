#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <math.h>

class NotchFilter {
public:
    NotchFilter() {}

    // center_freq: Frequência do ruído do motor (ex: 150 Hz)
    // bandwidth: Largura do corte (ex: 50 Hz)
    // dt: Delta time médio do loop (ex: 0.004 para 250Hz)
    void init(float center_freq, float bandwidth, float dt) {
        float omega = 2.0f * M_PI * center_freq * dt;
        float sn = sin(omega);
        float cs = cos(omega);
        float alpha = sn * sinh(M_LN2 / 2.0f * bandwidth * omega / sn);

        float a0 = 1.0f + alpha;
        b0 = 1.0f / a0;
        b1 = -2.0f * cs / a0;
        b2 = 1.0f / a0;
        a1 = -2.0f * cs / a0;
        a2 = (1.0f - alpha) / a0;

        x1 = x2 = y1 = y2 = 0.0f;
    }

    float apply(float sample) {
        float result = b0 * sample + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        x2 = x1;
        x1 = sample;
        y2 = y1;
        y1 = result;

        return result;
    }

private:
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
};

#endif