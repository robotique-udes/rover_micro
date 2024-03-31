#ifndef __LED_CONTROLLER_HPP__
#define __LED_CONTROLLER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include "driver/ledc.h"
#include "rover_helpers/helpers.hpp"

/// @brief Helper wrapper class to make it easier to blink led
/// @example The follwing code will blink the onboard led at 1Hz with a 
//  duty_cycle of 50%
///     int main(void)
///     {
///         LedBlinker ledBuildIn((gpio_num_t)2u, LEDC_TIMER_0, LEDC_CHANNEL_0);
///         ledBuildIn.init(1u, 50.0f);
///     }
class LedBlinker
{
public:
    /// @brief LedBlinker constructor, it's the user's job to make sure to only 
    /// connect one element to each timer or channel
    /// @param ledPin_ GPIO connected to the led, example for D2: (gpio_num_t)2
    /// @param timerNumber_ Ranges from 0 to 4, example LEDC_TIMER_3
    /// @param channelNumber_ Ranges from 0 to 8, example LEDC_CHANNEL_7
    LedBlinker(gpio_num_t ledPin_, ledc_timer_t timerNumber_ = LEDC_TIMER_3, ledc_channel_t channelNumber_ = LEDC_CHANNEL_7);
    ~LedBlinker();

    /// @brief Starts the internal timer on the gpio at specified frequency and duty cycle
    /// @param frequencyHz_ Frequency in Hz
    /// @param dutyCycle_ Duty Cycle in percent [0.0f to 100.0f]
    void init(uint32_t frequencyHz_, float dutyCycle_);

    /// @brief Set the led always on 
    void setOn(void);

    /// @brief Set the led always off
    void setOff(void);

    /// @brief Update the led flashing pattern
    /// @param frequencyHz_ Frequency in Hz
    /// @param dutyCycle_ Duty Cycle in percent [0.0f to 100.0f]
    void setBlink(uint32_t frequencyHz_, float dutyCycle_);

private:
    gpio_num_t _ledPin;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
};

#endif // defined(ESP32)
#endif // __LED_CONTROLLER_HPP__
