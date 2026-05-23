#include "dbutton.h"
#include <esp_timer.h>

// Constructor
DebouncedButton::DebouncedButton(gpio_num_t p, uint64_t debounce_us)  {
    pin = p;
    debounce_time_us = debounce_us; 
    stable_state = true;
    last_reading = true;
    last_change_time = 0;
}

void DebouncedButton::init() {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    reset();
}

void DebouncedButton::reset() {
    stable_state = gpio_get_level(pin);
    last_reading = stable_state;
    last_change_time = esp_timer_get_time();
}

bool DebouncedButton::isPressed() {
    uint64_t now = esp_timer_get_time();
    bool reading = gpio_get_level(pin);

    if (reading != last_reading)
        last_change_time = now; 

    if ((now - last_change_time) > debounce_time_us)
        stable_state = reading;
    
    last_reading = reading;

    // Return true if the stable state is 0 (Pressed due to pull-up)
    return (stable_state == 0); 
}