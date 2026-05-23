#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>

class DebouncedButton {
private:
    gpio_num_t pin;
    uint64_t debounce_time_us;
    bool stable_state;
    bool last_reading;
    uint64_t last_change_time;

public:
    // Constructor
    DebouncedButton(gpio_num_t p, uint64_t debounce_us = 40000);

    void init(); // Initialize the hardware pin
    void reset(); // Call this once when entering a state to clear stale readings
    bool isPressed(); // Call this continuously in your state loop. Returns true if firmly pressed.
};