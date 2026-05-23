#include "stepper28.h"

Stepper28BYJ::Stepper28BYJ(gpio_num_t p0, gpio_num_t p1, gpio_num_t p2, gpio_num_t p3, uint32_t delayUs) {
    pins[0] = p0; pins[1] = p1; pins[2] = p2; pins[3] = p3;
    stepDelayUs = delayUs;
    currentStep = 0;
    lastTime    = 0;
    march       = 0;
}

void Stepper28BYJ::begin() {
    for (int i = 0; i < NPINS; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
    lastTime = esp_timer_get_time();
}

bool Stepper28BYJ::update() {
    uint64_t now = esp_timer_get_time();
    if (now - lastTime >= stepDelayUs) {
        for (uint8_t i = 0; i < NPINS; i++) {
            bool on = seq[march][currentStep] & (1 << i);
            gpio_set_level(pins[i], on ? 1 : 0);
        }
        currentStep = (currentStep + 1) % NSTEPS;
        lastTime = now;
        return true;
    }
    return false;
}

void Stepper28BYJ::setDirection(bool forward) {
    march = forward ? 0 : 1;
}

void Stepper28BYJ::setDelay(uint32_t delayUs) {
    stepDelayUs = delayUs;
}
