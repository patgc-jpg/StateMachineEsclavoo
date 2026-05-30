#pragma once
#include <stdint.h>
#include <driver/gpio.h>
#include <esp_timer.h>

class Stepper28BYJ {
public:
    Stepper28BYJ(gpio_num_t p0, gpio_num_t p1, gpio_num_t p2, gpio_num_t p3, uint32_t delayUs);

    void begin();
    bool update();                    // Retorna true si dio un paso
    void setDirection(bool forward);  // true = adelante, false = reversa
    void setDelay(uint32_t delayUs);
    void setHighTorque(bool enabled); // true = usa secuencia de alto torque al avanzar

private:
    static const uint8_t NPINS = 4;
    static const uint8_t NSTEPS = 4;

    gpio_num_t pins[NPINS];
    uint32_t   stepDelayUs;
    uint8_t    currentStep;
    uint64_t   lastTime;
    uint8_t    march;       // 0 = adelante, 1 = reversa
    bool       highTorque;  // si true y march==0, usa fila 2 (alto torque)

    const uint8_t seq[3][NSTEPS] = {
        { 0x01, 0x02, 0x04, 0x08 },   // 0: adelante (normal)
        { 0x08, 0x04, 0x02, 0x01 },   // 1: reversa
        { 0x03, 0x06, 0x0C, 0x09 },   // 2: adelante alto torque (half-step)
    };
};
