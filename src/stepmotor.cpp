#include "stepmotor.h"
#include <esp_rom_sys.h>   // esp_rom_delay_us

StepperMotor::StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, uint32_t delayUs)
    : _stepPin(stepPin), _dirPin(dirPin), stepDelayUs(delayUs), lastTime(0), _forward(true) {}

void StepperMotor::begin() {
    gpio_reset_pin(_stepPin);
    gpio_set_direction(_stepPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_stepPin, 0);

    gpio_reset_pin(_dirPin);
    gpio_set_direction(_dirPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_dirPin, _forward ? 1 : 0);

    lastTime = esp_timer_get_time();
}

// Genera un pulso completo STEP por cada llamada cuando el timer lo permite.
// DRV8825 avanza en el flanco ascendente; ancho mínimo de pulso = 1.9 µs.
bool StepperMotor::update() {
    uint64_t now = esp_timer_get_time();
    if (now - lastTime >= stepDelayUs) {
        esp_rom_delay_us(1);          // garantiza TSET >= 650 ns del DRV8825 tras cambio de DIR
        gpio_set_level(_stepPin, 1);
        esp_rom_delay_us(2);          // cumple el mínimo de 1.9 µs del DRV8825
        gpio_set_level(_stepPin, 0);
        lastTime = now;
        return true;
    }
    return false;
}

void StepperMotor::setDirection(bool forward) {
    if (_forward == forward) return;  // evita escritura GPIO innecesaria si no cambia
    _forward = forward;
    gpio_set_level(_dirPin, forward ? 1 : 0);
}

void StepperMotor::resetTimer() {
    lastTime = esp_timer_get_time();
}

void StepperMotor::ReverseDirec() {
    setDirection(!_forward);
}

void StepperMotor::setDelay(uint32_t delayUs) {
    stepDelayUs = delayUs;
}
