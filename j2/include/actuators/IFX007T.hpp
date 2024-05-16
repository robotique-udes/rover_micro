#ifndef __IFX007T_HPP__
#define __IFX007T_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include "Arduino.h"
#include "driver/ledc.h"
#include "rover_helpers/macros.hpp"
#include "actuators/motor_driver.hpp"
#include "rover_helpers/assert.hpp"
#include "rover_helpers/led_blinker.hpp"

// PWM frequency should be 10 times rise-time as per datasheet
// Rise time with SR = 0: 0.25us

class IFX007T : protected MotorDriver
{
public:
    static constexpr uint32_t PWM_FREQUENCY = 1'000;
    static constexpr ledc_timer_bit_t LEDC_TIMER_RESOLUTION = LEDC_TIMER_14_BIT;

    static constexpr uint32_t PERCENT_TO_DUTY(float percent_)
    {
        return (uint32_t)round(abs(percent_) / 100.0f * (float)(1u << LEDC_TIMER_RESOLUTION) - 1.0f);
    }

    IFX007T(){};
    virtual ~IFX007T(){};

    void init(gpio_num_t en_1_,
              gpio_num_t en_2_,
              gpio_num_t in_1_,
              gpio_num_t in_2_,
              MotorDriver::eBrakeMode brakeMode_ = MotorDriver::eBrakeMode::COAST,
              bool reversed_ = false,
              ledc_timer_t timerNumber_ = LEDC_TIMER_0,
              ledc_channel_t channelNumber1_ = LEDC_CHANNEL_0,
              ledc_channel_t channelNumber2_ = LEDC_CHANNEL_1)
    {
        ASSERT(en_1_ == GPIO_NUM_NC);
        ASSERT(en_2_ == GPIO_NUM_NC);
        ASSERT(in_1_ == GPIO_NUM_NC);
        ASSERT(in_2_ == GPIO_NUM_NC);
        _en_1 = en_1_;
        _en_2 = en_2_;
        _in_1 = in_1_;
        _in_2 = in_2_;
        pinMode(_en_1, OUTPUT);
        pinMode(_en_2, OUTPUT);
        pinMode(_in_1, OUTPUT);
        pinMode(_in_2, OUTPUT);

        _brakeMode = brakeMode_;

        _reversed = reversed_;

        _ledc_motorTimer = timerNumber_;

        ASSERT(channelNumber1_ == channelNumber2_, "Using the same channel for both half-bridge will result in constant braking of the motor")
        _ledc_motorChannel_1 = channelNumber1_;
        _ledc_motorChannel_2 = channelNumber2_;

        // Timer for PWM
        _freqPWM = PWM_FREQUENCY;
        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _ledc_motorTimer;
        ledc_timer.duty_resolution = LEDC_TIMER_RESOLUTION;
        ledc_timer.freq_hz = PWM_FREQUENCY;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        // Channel for 1st bridge
        ledc_channel_config_t ledc_channel_1;
        ledc_channel_1.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel_1.channel = _ledc_motorChannel_1;
        ledc_channel_1.timer_sel = _ledc_motorTimer;
        ledc_channel_1.intr_type = LEDC_INTR_DISABLE;
        ledc_channel_1.gpio_num = _in_1;
        ledc_channel_1.duty = PERCENT_TO_DUTY(0.0f);
        ledc_channel_1.hpoint = 0;
        ledc_channel_config(&ledc_channel_1);

        // Channel for 2nd bridge (on same timer as 1st)
        ledc_channel_config_t ledc_channel_2;
        ledc_channel_2.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel_2.channel = _ledc_motorChannel_2;
        ledc_channel_2.timer_sel = _ledc_motorTimer;
        ledc_channel_2.intr_type = LEDC_INTR_DISABLE;
        ledc_channel_2.gpio_num = _in_2;
        ledc_channel_2.duty = PERCENT_TO_DUTY(0.0f);
        ledc_channel_2.hpoint = 0;
        ledc_channel_config(&ledc_channel_2);

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_motorTimer, _freqPWM);

        // Flashing EN pins low for 10us to reset termal shutdown if necessary
        // min t_reset = 4us as per datasheet
        digitalWrite(_en_1, LOW);
        digitalWrite(_en_2, LOW);
        delayMicroseconds(10);

        this->enable();
        this->setSpeed(100.0f);
    }

    void setSpeed(float spd_)
    {
        if (!_enabled)
        {
            LOG(WARN, "Motor is disabled, can't set speed");
            return;
        }

        spd_ = constrain(spd_, -100.0f, 100.0f);
        _currentSpd = spd_;

        if (IN_ERROR(spd_, 0.001f, 0.0f))
        {
            _currentSpd = 0.0f;
            // Brake
            if (_brakeMode == MotorDriver::eBrakeMode::BRAKE)
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(0.0f));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(0.0f));
            }
            else if (_brakeMode == MotorDriver::eBrakeMode::COAST)
            {
                digitalWrite(_en_1, LOW);
                digitalWrite(_en_2, LOW);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(0.0f));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(0.0f));
            }
            else
            {
                ASSERT(true, "Shouldn't ever fall ever, implementation error... aborting");
            }
        }
        else if (spd_ > 0.0f)
        {
            digitalWrite(_en_1, HIGH);
            digitalWrite(_en_2, HIGH);
            if (!_reversed)
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(spd_));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(0.0f));
            }
            else
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(0.0f));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(spd_));
            }
        }
        else if (spd_ < 0.0f)
        {
            digitalWrite(_en_1, HIGH);
            digitalWrite(_en_2, HIGH);
            if (!_reversed)
            {
                LOG(INFO, "Here");
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(0.0f));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(spd_));
            }
            else
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, PERCENT_TO_DUTY(spd_));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, PERCENT_TO_DUTY(0.0f));
            }
        }
        else
        {
            ASSERT(true, "Shouldn't ever fall ever, implementation error... aborting");
        }

        ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2);
        this->updateLed();
    }

    void enable(void)
    {
        if (_enabled)
        {
            return;
        }

        if (_brakeMode == MotorDriver::eBrakeMode::BRAKE)
        {
            digitalWrite(_en_1, HIGH);
            digitalWrite(_en_2, HIGH);
        }

        _enabled = true;
    }

    void disable(void)
    {
        if (!_enabled)
        {
            return;
        }

        this->setSpeed(0.0f);

        digitalWrite(_en_1, LOW);
        digitalWrite(_en_2, LOW);

        _enabled = false;
    }

    void reset(void)
    {
        this->disable();
        this->enable();
    }

    bool isMoving(void)
    {
        if (_enabled && IN_ERROR(_currentSpd, 0.01f, 0.0f))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void attachRGBLed(gpio_num_t ledR_,
                      gpio_num_t ledG_,
                      gpio_num_t ledB_,
                      ledc_channel_t ledcChannelR_ = LEDC_CHANNEL_4,
                      ledc_channel_t ledcChannelG_ = LEDC_CHANNEL_5,
                      ledc_channel_t ledcChannelB_ = LEDC_CHANNEL_6,
                      ledc_timer_t ledTimer_ = LEDC_TIMER_3)
    {
        _ledR = ledR_;
        _ledG = ledG_;
        _ledB = ledB_;

        _ledc_ledTimer = ledTimer_;
        _ledc_ledChannelR = ledcChannelR_;
        _ledc_ledChannelG = ledcChannelG_;
        _ledc_ledChannelB = ledcChannelB_;

        if (_ledR != GPIO_NUM_NC &&
            _ledG != GPIO_NUM_NC &&
            _ledB != GPIO_NUM_NC)
        {
            _withLed = true;
        }

        if (!_withLed)
        {
            return;
        }

        pinMode(_ledR, OUTPUT);
        pinMode(_ledG, OUTPUT);
        pinMode(_ledB, OUTPUT);

        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _ledc_ledTimer;
        ledc_timer.duty_resolution = LEDC_TIMER_RESOLUTION;
        ledc_timer.freq_hz = 1000;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channelR;
        ledc_channelR.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channelR.channel = _ledc_ledChannelR;
        ledc_channelR.timer_sel = _ledc_ledTimer;
        ledc_channelR.intr_type = LEDC_INTR_DISABLE;
        ledc_channelR.gpio_num = _ledR;
        ledc_channelR.duty = PERCENT_TO_DUTY(10.0f);
        ledc_channelR.hpoint = 0;
        ledc_channel_config(&ledc_channelR);

        ledc_channel_config_t ledc_channelG;
        ledc_channelG.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channelG.channel = _ledc_ledChannelG;
        ledc_channelG.timer_sel = _ledc_ledTimer;
        ledc_channelG.intr_type = LEDC_INTR_DISABLE;

        if (_brakeMode == MotorDriver::eBrakeMode::BRAKE)
        {
            ledc_channelG.gpio_num = _ledG;
        }
        else
        {
            ledc_channelG.gpio_num = _ledB;
        }
        ledc_channelG.duty = PERCENT_TO_DUTY(2.0f);
        ledc_channelG.hpoint = 0;
        ledc_channel_config(&ledc_channelG);
    }

private:
    gpio_num_t _en_1 = GPIO_NUM_NC;
    gpio_num_t _en_2 = GPIO_NUM_NC;
    gpio_num_t _in_1 = GPIO_NUM_NC;
    gpio_num_t _in_2 = GPIO_NUM_NC;

    gpio_num_t _ledR = GPIO_NUM_NC;
    gpio_num_t _ledG = GPIO_NUM_NC;
    gpio_num_t _ledB = GPIO_NUM_NC;

    uint32_t _freqPWM;
    bool _reversed = false;
    bool _withLed = false;

    ledc_timer_t _ledc_motorTimer;
    ledc_channel_t _ledc_motorChannel_1;
    ledc_channel_t _ledc_motorChannel_2;

    ledc_timer_t _ledc_ledTimer;
    ledc_channel_t _ledc_ledChannelR;
    ledc_channel_t _ledc_ledChannelG;
    ledc_channel_t _ledc_ledChannelB;

    MotorDriver::eBrakeMode _brakeMode;
    bool _enabled = false;
    float _currentSpd = 0.0f;

    void updateLed()
    {
        if (_currentSpd > 99.9f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 100);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(5.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > 75.0f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 16);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > 50.0f)
        {
            LOG(INFO, "%i", ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 8));
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > 25.0f)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(50.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
            LOG(WARN, "%i", ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 1));
        }
        else if (_currentSpd > 0.01f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > -0.01f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 1000);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(2.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > -25.0f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > -50.0f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 2);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > -75.0f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 4);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
        else if (_currentSpd > -100.0f)
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_ledTimer, 8);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR, PERCENT_TO_DUTY(0.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelR);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG, PERCENT_TO_DUTY(3.0f));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_ledChannelG);
        }
    }
};

#endif // !defined(ESP32)
#endif // __IFX007T_HPP__
