#ifndef __SOFT_LED_BLINKER_HPP__
#define __SOFT_LED_BLINKER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/log.hpp"
#include "rover_helpers/timer.hpp"
#include "rover_helpers/chrono.hpp"

namespace RoverHelpers
{

    /// @brief Helper wrapper class to make it easier to blink led (software based)
    /// @example The follwing code will blink the onboard led at 1Hz with a
    /// duty_cycle of 50% and 20% brightness
    ///
    ///     int main(void)
    ///     {
    ///         RoverHelpers::SoftLedBlinker ledBuildIn;
    ///         ledBuildIn.init(GPIO_NUM_2, 20.0f, 1);
    ///
    ///         for (;;)
    ///         {
    ///             ledBuildIn.update();
    ///         }
    ///     }
    class SoftLedBlinker
    {
    public:
        SoftLedBlinker(void) {}
        ~SoftLedBlinker(void) {}

        /// @brief Needs to be called for led to work
        /// @param ledPin_ GPIO attached to LED
        /// @param brightness_ Value in percent [0->100]
        /// @param blinkingFreq_ Blinking frequency [0->50]
        void init(gpio_num_t ledPin_, float brightness_ = 100.0f, uint16_t blinkingFreq_ = 0);

        /// @brief Change the blinking frequency
        /// @param frequency_ Blinking frequency [0->50]
        void setBlink(uint8_t frequency_);

        /// @brief Change the LED brightness
        /// @param brightness_ Value in percent [0->100]
        void setBrightness(float brightness_);

        // Need to be called as often as possible
        void update(void);

    private:
        gpio_num_t _ledPin = GPIO_NUM_NC;
        float _highPulseTime = 100.0f;
        bool _highCycle = LOW;
        uint8_t _frequency = 0.0f;
        uint8_t _brightness = 0.0f;


        RoverHelpers::Timer<unsigned long, micros> _timerBrightness = RoverHelpers::Timer<unsigned long, micros>(100u);
        RoverHelpers::Chrono<unsigned long, micros> _chronoHighPulse;
        RoverHelpers::Timer<unsigned long, millis> _timerBlinking = RoverHelpers::Timer<unsigned long, millis>(20u);
    };

    void SoftLedBlinker::init(gpio_num_t ledPin_, float brightness_, uint16_t blinkingFreq_)
    {
        _ledPin = ledPin_;

        pinMode(_ledPin, OUTPUT);

        this->setBlink(blinkingFreq_);
        this->setBrightness(brightness_);
    }

    void SoftLedBlinker::setBlink(uint8_t frequency_)
    {
        _frequency = constrain(frequency_, 0, 50);
        _timerBlinking.updateInterval((unsigned long)(500.0f / (float)_frequency));
    }

    void SoftLedBlinker::setBrightness(float brightness_)
    {
        _brightness = constrain(brightness_, 0.0f, 100.0f);
        _highPulseTime = _brightness/100.0f * _timerBrightness.getInterval();

        if (brightness_ == 0.0f)
        {
            digitalWrite(_ledPin, LOW);
        }
    }

    void SoftLedBlinker::update(void)
    {
        if (_brightness == 0.0f)
        {
            return;
        }

        if (_timerBrightness.isDone() && _highCycle)
        {
            digitalWrite(_ledPin, HIGH);
            _chronoHighPulse.restart();
        }

        if (_chronoHighPulse.getTime() > _highPulseTime)
        {
            digitalWrite(_ledPin, LOW);
        }

        if (_timerBlinking.isDone())
        {
            _highCycle = !_highCycle;
        }
    }
}

#endif // ESP32
#endif // __SOFT_LED_BLINKER_HPP__
