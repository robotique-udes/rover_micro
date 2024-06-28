#ifndef __IFX007T_HPP__
#define __IFX007T_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include "Arduino.h"
#include "driver/ledc.h"
#include "rover_helpers/macros.hpp"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "rover_helpers/assert.hpp"

class IFX007T : public MotorDriver
{
public:
    static constexpr uint32_t PWM_FREQUENCY = 1'000;
    static constexpr ledc_timer_bit_t LEDC_TIMER_RESOLUTION = LEDC_TIMER_14_BIT;

    static constexpr uint32_t PERCENT_TO_DUTY(float percent_)
    {
        return (uint32_t)round(abs(percent_) / 100.0f * (float)(1u << LEDC_TIMER_RESOLUTION) - 1.0f);
    }

    IFX007T(gpio_num_t en_1_,
            gpio_num_t en_2_,
            gpio_num_t in_1_,
            gpio_num_t in_2_,
            MotorDriver::eBrakeMode brakeMode_ = MotorDriver::eBrakeMode::COAST,
            bool reversed_ = false,
            ledc_timer_t timerNumber_ = LEDC_TIMER_0,
            ledc_channel_t channelNumber1_ = LEDC_CHANNEL_0,
            ledc_channel_t channelNumber2_ = LEDC_CHANNEL_1)
        : MotorDriver(eBrakeMode::NONE)
    {
        ASSERT(brakeMode_ != eBrakeMode::COAST && brakeMode_ != eBrakeMode::BRAKE,
               "IFX007T doesn't support specified brake mode");
        this->setBrakeMode(brakeMode_);

        ASSERT(en_1_ == GPIO_NUM_NC);
        ASSERT(en_2_ == GPIO_NUM_NC);
        ASSERT(in_1_ == GPIO_NUM_NC);
        ASSERT(in_2_ == GPIO_NUM_NC);
        _en_1 = en_1_;
        _en_2 = en_2_;
        _in_1 = in_1_;
        _in_2 = in_2_;

        _reversed = reversed_;

        _ledc_motorTimer = timerNumber_;
        ASSERT(channelNumber1_ == channelNumber2_,
         "Using the same channel for both half-bridge will result in constant braking of the motor")
        _ledc_motorChannel_1 = channelNumber1_;
        _ledc_motorChannel_2 = channelNumber2_;
    };

    virtual ~IFX007T()
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, 0u);
        ledc_stop(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_2, 0u);
    };

    void init()
    {
        pinMode(_en_1, OUTPUT);
        pinMode(_en_2, OUTPUT);
        pinMode(_in_1, OUTPUT);
        pinMode(_in_2, OUTPUT);

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
        ledc_channel_1.flags.output_invert = false;
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
        ledc_channel_2.flags.output_invert = false;
        ledc_channel_config(&ledc_channel_2);

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_motorTimer, _freqPWM);

        // Flashing EN pins low for 10us to reset termal shutdown if necessary
        // min t_reset = 4us as per datasheet
        digitalWrite(_en_1, LOW);
        digitalWrite(_en_2, LOW);
        delayMicroseconds(10);

        this->initDone();

        this->enable();
        this->setCmd(0.0f);
    }

    void setCmdInternal(float spd_)
    {
        this->checkInit();

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
    }

    void enable(void)
    {
        this->checkInit();

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
        this->checkInit();

        if (!_enabled)
        {
            return;
        }

        this->setCmd(0.0f);

        digitalWrite(_en_1, LOW);
        digitalWrite(_en_2, LOW);

        _enabled = false;
    }

    void reset(void)
    {
        this->checkInit();

        this->disable();
        this->enable();
    }

    bool isMoving(void)
    {
        this->checkInit();

        if (_enabled && IN_ERROR(_currentSpd, 0.01f, 0.0f))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void updateInternal(void)
    {
        // Not needed for IFX007T*
    }

    float getCmd(void)
    {
        return _currentSpd;
    }

    void setBrakeMode(eBrakeMode brakeMode_)
    {
        if (brakeMode_ == eBrakeMode::BRAKE || brakeMode_ == eBrakeMode::COAST)
        {
            _brakeMode = brakeMode_;
        }
        else
        {
            LOG(WARN, "Wrong brake parameters for motor driver IFX007T");
        }
    }

private:
    gpio_num_t _en_1 = GPIO_NUM_NC;
    gpio_num_t _en_2 = GPIO_NUM_NC;
    gpio_num_t _in_1 = GPIO_NUM_NC;
    gpio_num_t _in_2 = GPIO_NUM_NC;

    uint32_t _freqPWM;
    bool _reversed = false;

    ledc_timer_t _ledc_motorTimer;
    ledc_channel_t _ledc_motorChannel_1;
    ledc_channel_t _ledc_motorChannel_2;

    bool _enabled = false;
    float _currentSpd = 0.0f;
};

#endif // !defined(ESP32)
#endif // __IFX007T_HPP__
