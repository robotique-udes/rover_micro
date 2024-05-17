#ifndef __LIMIT_SWITCH_H__
#define __LIMIT_SWITCH_H__

#include <stdint.h>
#include "Arduino.h"
#include "rover_helpers/log.hpp"

class LimitSwitch
{
public:
    enum class eLimitSwitchMode
    {
        PullUp,
        PullUpInternal,
        PullDown
    };

    struct sSwitchParams
    {
        eLimitSwitchMode MODE;
        gpio_num_t PIN;

        sSwitchParams(eLimitSwitchMode mode_, gpio_num_t pin_)
        {
            MODE = mode_;
            PIN = pin_;
        }
    };

private:
    eLimitSwitchMode _mode;
    uint8_t _pin;

public:
    LimitSwitch();
    ~LimitSwitch();

    void init(sSwitchParams);
    void init(eLimitSwitchMode mode, gpio_num_t pinCom);
    bool isClicked();
};

LimitSwitch::LimitSwitch()
{
}

LimitSwitch::~LimitSwitch()
{
}

void LimitSwitch::init(sSwitchParams switchParams)
{
    this->init(switchParams.MODE, switchParams.PIN);
}

void LimitSwitch::init(eLimitSwitchMode mode, gpio_num_t pinCom)
{
    _mode = mode;
    _pin = pinCom;

    if (_mode == eLimitSwitchMode::PullUpInternal)
    {
        pinMode(_pin, INPUT_PULLUP);
    }
    else
    {
        pinMode(_pin, INPUT);
    }
}

bool LimitSwitch::isClicked()
{
    return (_mode == eLimitSwitchMode::PullUp) ? !digitalRead(_pin) : digitalRead(_pin);
}

#endif
