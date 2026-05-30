#pragma once
#include "driver/ledc.h"

class Solenoid {
public:
    Solenoid(int gpio,
             ledc_channel_t channel = LEDC_CHANNEL_0,
             ledc_timer_t   timer   = LEDC_TIMER_0);

    void begin();       // inicializa PWM — llamar una vez en app_main
    void highTorque();  // ~90% duty → agarre fuerte
    void lowTorque();   // ~60% duty → agarre suave
    void open();        // 0% duty  → solenoide suelto

private:
    int            _gpio;
    ledc_channel_t _channel;
    ledc_timer_t   _timer;

    static constexpr uint32_t          FREQ_HZ    = 150;
    static constexpr ledc_timer_bit_t  RESOLUTION = LEDC_TIMER_10_BIT;
    static constexpr uint32_t          DUTY_OFF   = 0;
    static constexpr uint32_t          DUTY_LOW   = 614;  // ~60% de 1023
    static constexpr uint32_t          DUTY_HIGH  = 921;  // ~90% de 1023

    void setDuty(uint32_t duty);
};
