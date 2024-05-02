#ifndef __TALONSRX_DRIVE_HPP__
#define __TALONSRX_DRIVE_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include "Arduino.h"
#include "driver/ledc.h"
#include "actuators/motor_driver.hpp"
#include "rover_helpers/log.hpp"

namespace TalonSrxConstant
{
    constexpr float SIGNAL_FULL_STOP_MS = 1500.0f; // Middle of deadband
    constexpr float SIGNAL_FULL_REVERSE_MS = 1000.0f;
    constexpr float SIGNAL_MIN_REVERSE_MS = 1480.0f;
    constexpr float SIGNAL_FULL_FORWARD_MS = 2000.0f;
    constexpr float SIGNAL_MIN_FORWARD_MS = 1520.0f;

    constexpr uint32_t MS_TO_DUTY(uint32_t freq_, float microSeconds_)
    {
        float period = 1.0f / static_cast<float>(freq_);
        return static_cast<uint32_t>(round(((microSeconds_ / 1'000'000.f) / period) * 1048576.0f));
    }
}

class TalonSrx : public MotorDriver
{
public:
    // Try using the same timer for all same frequency signal. Don't use the same channel
    TalonSrx(gpio_num_t pinPWM_, ledc_timer_t timerNumber_ = LEDC_TIMER_0, ledc_channel_t channelNumber_ = LEDC_CHANNEL_0)
    {
        _pinPWM = pinPWM_;
        _timer = timerNumber_;
        _channel = channelNumber_;
    }

    ~TalonSrx()
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _channel, 0u);
    }

    void init(float microseconds_ = TalonSrxConstant::SIGNAL_FULL_STOP_MS, float signalFrequencyHz_ = 50.0F)
    {
        _freqSignal = (uint32_t)round(signalFrequencyHz_);
        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _timer;
        ledc_timer.duty_resolution = LEDC_TIMER_20_BIT;
        ledc_timer.freq_hz = _freqSignal;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel;
        ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel = _channel;
        ledc_channel.timer_sel = _timer;
        ledc_channel.intr_type = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num = _pinPWM;
        ledc_channel.duty = TalonSrxConstant::MS_TO_DUTY(_freqSignal, microseconds_);
        ledc_channel.hpoint = 0;
        ledc_channel_config(&ledc_channel);

        ASSERT(_freqSignal == 0u || _freqSignal < 10u || _freqSignal > 200u, "Invalid signal frequency for TalonSrx")

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, _freqSignal);
        this->writeMicroseconds(microseconds_);
    }

    // -100 to 100 for speed
    void setSpd(float spd)
    {
        if (spd < -_maxSpeed || spd > _maxSpeed)
        {
            LOG(WARN, "Speed value exceed max power: %f", spd);
            _spd = (spd < -_maxSpeed) ? -_maxSpeed : _maxSpeed;
        }
        else
        {
            _spd = spd;
        }
        writeMicroseconds(MAP(_spd, -100.0f, 100.0f, 1000.0f, 2000.0f));
    }

    void setSpd(float spd, bool override_)
    {
        if (override_ == true)
        {
            if (spd < -_maxSpeed || spd > _maxSpeed)
            {
                LOG(WARN, "Speed value exceed max power: %f", spd);
            }
            writeMicroseconds(MAP(spd, -100.0f, 100.0f, 1000.0f, 2000.0f));
        }
        else
        {
            LOG(WARN, "Cant't override max speed: %f", spd);
        }
    }

    void enableMot(void) { /*code to enable the motor if applied.*/ }
    void disableMot(void)
    {
        stop();
    }
    void resetMot(void) { /*code to reset the motor if applied.*/ }

    // Return true if the motor is moving.
    bool isMoving(void)
    {
        if (_spd != 0)
            return true;
        else
            return false;
    }

    // Set max dutycycle from 0-100% to limit the output power.
    void setMaxSpeed(uint8_t spd)
    {
        _maxSpeed = spd;
        LOG(INFO, "Max speed set at : %i", spd);
    }

private:
    void writeMicroseconds(float microseconds_)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, TalonSrxConstant::MS_TO_DUTY(_freqSignal, microseconds_));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    void stop()
    {
        writeMicroseconds(TalonSrxConstant::SIGNAL_FULL_STOP_MS);
    }

    uint8_t _maxSpeed = 64;
    gpio_num_t _pinPWM;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    uint32_t _freqSignal;
    float _spd;
};

#endif // !defined(ESP32)
#endif // __DC_MOTOR_HPP__
