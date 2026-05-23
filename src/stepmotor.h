#pragma once

#include <stdint.h>
#include <driver/gpio.h>
#include <esp_timer.h>

class StepperMotor {
public:
    StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, uint32_t delayUs);

    void begin();
    bool update();                   // Non-blocking; returns true if a step actually fired
    void setDelay(uint32_t delayUs);
    void setDirection(bool forward); // true = forward (DIR=HIGH), false = reverse (DIR=LOW)
    void ReverseDirec();

private:
    gpio_num_t _stepPin;
    gpio_num_t _dirPin;
    uint32_t   stepDelayUs;
    uint64_t   lastTime;
    bool       _forward;
};
