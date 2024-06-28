#ifndef __TALONSRX_DRIVE_HPP__
#define __TALONSRX_DRIVE_HPP__

#if !defined(ARDUINO_ESP32S2_DEV) && !defined(ARDUINO_ESP32S3_DEV)
#error CPU is not supported
#else

#include "Arduino.h"
#include "driver/ledc.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "rover_helpers/helpers.hpp"

class TalonSrx : public MotorDriver
{
private:
    static constexpr float SIGNAL_FULL_STOP_MS = 1500.0f; // Middle of deadband
    static constexpr float SIGNAL_FULL_REVERSE_MS = 1000.0f;
    static constexpr float SIGNAL_MIN_REVERSE_MS = 1480.0f;
    static constexpr float SIGNAL_FULL_FORWARD_MS = 2000.0f;
    static constexpr float SIGNAL_MIN_FORWARD_MS = 1520.0f;

    static constexpr uint32_t MS_TO_DUTY(uint32_t freq_, float microSeconds_)
    {
        float period = 1.0f / static_cast<float>(freq_);
        return static_cast<uint32_t>(round(((microSeconds_ / 1'000'000.f) / period) * 1048576.0f));
    }

public:
    // Try using the same timer for all same frequency signal. Don't use the same channel
    TalonSrx(gpio_num_t pinPWM_,
             ledc_timer_t timerNumber_ = LEDC_TIMER_0,
             ledc_channel_t channelNumber_ = LEDC_CHANNEL_0,
             float signalFrequencyHz_ = 50.0f)
        : MotorDriver(eBrakeMode::NONE)
    {
#error The TalonSRX class has been updated and cleaned blindly (no available working board to test on): please take the time to test it before using

        _pinPWM = pinPWM_;
        _timer = timerNumber_;
        _channel = channelNumber_;

        _freqSignal = (uint32_t)round(signalFrequencyHz_);
    }

    ~TalonSrx()
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _channel, 0u);
    }

    void init(void)
    {
        pinMode(_pinPWM, OUTPUT);

        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _timer;
#if defined(ARDUINO_ESP32S2_DEV)
        ledc_timer.duty_resolution = LEDC_TIMER_20_BIT;
#elif defined(ARDUINO_ESP32S3_DEV)
        ledc_timer.duty_resolution = LEDC_TIMER_14_BIT;
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
        ledc_channel.duty = MS_TO_DUTY(_freqSignal, SIGNAL_FULL_STOP_MS);
        ledc_channel.hpoint = 0;
        ledc_channel_config(&ledc_channel);

        ASSERT(_freqSignal < 10u || _freqSignal > 200u, "Invalid signal frequency for TalonSrx")

        this->initDone();

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, _freqSignal);
        this->writeMicroseconds(SIGNAL_FULL_STOP_MS);
    }

    void updateInternal(void)
    {
        // No update needed but should be called
    }

    // -100.0 to 100.0 for speed
    void setCmdInternal(float cmd_)
    {
        _speed = cmd_;

        if (IN_ERROR(_speed, 0.01f, 0.0f))
        {
            this->stop();
        }
        else if (_speed < 0.0f)
        {
            writeMicroseconds(MAP(_speed, -100.0f, 0.0f, SIGNAL_FULL_REVERSE_MS, SIGNAL_MIN_REVERSE_MS));
        }
        else if (_speed > 0.0f)
        {
            writeMicroseconds(MAP(_speed, 0.0f, 100.0f, SIGNAL_MIN_FORWARD_MS, SIGNAL_FULL_FORWARD_MS));
        }
    }

    float getCmd()
    {
        return _speed;
    }

    void enable(void)
    {
        LOG(INFO, "Enabling not supported on talonSRX");
    }

    void disable(void)
    {
        stop();
    }

    void reset(void)
    {
        LOG(INFO, "Resetting not supported on talonSRX");
    }

    // Return true if the motor is moving.
    bool isMoving(void)
    {
        if (_speed != 0.0f)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void setBrakeMode(eBrakeMode brakeMode_)
    {
        LOG(WARN, "TalonSRX brake mode can only be changed with the hardware button, refer to the TalonSRX's documentation")
    }

private:
    void writeMicroseconds(float microseconds_)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, MS_TO_DUTY(_freqSignal, microseconds_));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    void stop(void)
    {
        writeMicroseconds(SIGNAL_FULL_STOP_MS);
    }

    float _protectionSpeed = 0.0f;
    gpio_num_t _pinPWM;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    uint32_t _freqSignal;
    float _speed;
};

#endif // !defined(ARDUINO_ESP32S2_DEV) && !defined(ARDUINO_ESP32S3_DEV)
#endif // __DC_MOTOR_HPP__
