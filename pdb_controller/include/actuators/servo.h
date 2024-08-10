#ifndef __SERVO_HPP__
#define __SERVO_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include "Arduino.h"
#include "driver/ledc.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "rover_helpers/helpers.hpp"

// 500us to 2700us

class Servo
{
private:
    static constexpr uint32_t MS_TO_DUTY(uint32_t freq_, float microSeconds_)
    {
        float period = 1.0f / static_cast<float>(freq_);
#if defined(ARDUINO_ESP32S3_DEV)
        return static_cast<uint32_t>(round(((microSeconds_ / 1'000'000.f) / period) * ((float)(1u << 14))));
#else
        return static_cast<uint32_t>(round(((microSeconds_ / 1'000'000.f) / period) * ((float)(1u << 20))));
#endif
    }

public:
    // Try using the same timer for all same frequency signal. Don't use the same channel
    Servo(gpio_num_t pinPWM_,
          float minMicroseconds_ = 1000.0f,
          float maxMicroseconds_ = 2000.0f,
          float defaultPositionUS_ = 1500.0f,
          ledc_timer_t timerNumber_ = LEDC_TIMER_0,
          ledc_channel_t channelNumber_ = LEDC_CHANNEL_0,
          float signalFrequencyHz_ = 50.0f)
    {
        _pinPWM = pinPWM_;
        _timer = timerNumber_;
        _channel = channelNumber_;

        ASSERT(maxMicroseconds_ < minMicroseconds_)
        ASSERT(minMicroseconds_ < 0.0f || maxMicroseconds_ < 0.0f)
        ASSERT(minMicroseconds_ > defaultPositionUS_ || defaultPositionUS_ > maxMicroseconds_)

        _minPosUs = minMicroseconds_;
        _maxPosUs = maxMicroseconds_;
        _defaultPosUS = defaultPositionUS_;

        _freqSignal = (uint32_t)round(signalFrequencyHz_);
    }

    ~Servo(void)
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _channel, 0u);
    }

    void init(void)
    {
        pinMode(_pinPWM, OUTPUT);

        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _timer;
#if defined(ARDUINO_ESP32S3_DEV)
        ledc_timer.duty_resolution = LEDC_TIMER_14_BIT;
#else
        ledc_timer.duty_resolution = LEDC_TIMER_20_BIT;
#endif // defined(ARDUINO_ESP32S2_DEV)
        ledc_timer.freq_hz = _freqSignal;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel;
        ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel = _channel;
        ledc_channel.timer_sel = _timer;
        ledc_channel.intr_type = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num = _pinPWM;
        ledc_channel.duty = MS_TO_DUTY(_freqSignal, _defaultPosUS);
        ledc_channel.hpoint = 0;
        ledc_channel_config(&ledc_channel);

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, _freqSignal);
        this->writeMicroseconds(_defaultPosUS);
    }

    void update(void)
    {
        // No update needed but should be called
    }

    void writeMicroseconds(float microseconds_)
    {
        float cmdUS = constrain(microseconds_, _minPosUs, _maxPosUs);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, MS_TO_DUTY(_freqSignal, microseconds_));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

private:
    float _protectionSpeed = 0.0f;
    gpio_num_t _pinPWM;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    uint32_t _freqSignal;
    float _minPosUs;
    float _maxPosUs;
    float _defaultPosUS;
};

#endif // defined(ESP32)
#endif // __SERVO_HPP__
