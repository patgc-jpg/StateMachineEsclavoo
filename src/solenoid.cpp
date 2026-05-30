#include "solenoid.h"
#include "esp_err.h"

Solenoid::Solenoid(int gpio, ledc_channel_t channel, ledc_timer_t timer)
    : _gpio(gpio), _channel(channel), _timer(timer) {}

void Solenoid::begin()
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = RESOLUTION,
        .timer_num       = _timer,
        .freq_hz         = FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = _gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = _channel,
        .timer_sel  = _timer,
        .duty       = DUTY_OFF,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

void Solenoid::setDuty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
}

void Solenoid::highTorque() { setDuty(DUTY_HIGH); }
void Solenoid::lowTorque()  { setDuty(DUTY_LOW);  }
void Solenoid::open()       { setDuty(DUTY_OFF);  }
