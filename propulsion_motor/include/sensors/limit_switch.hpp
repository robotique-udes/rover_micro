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
    LimitSwitch(sSwitchParams);
    LimitSwitch(eLimitSwitchMode mode, gpio_num_t pinCom);
    ~LimitSwitch();

    void init();
    bool isClicked();
};

LimitSwitch::LimitSwitch(sSwitchParams switchParams) : LimitSwitch::LimitSwitch(switchParams.MODE, switchParams.PIN)
{
}

LimitSwitch::LimitSwitch(eLimitSwitchMode mode, gpio_num_t pinCom)
{
    _mode = mode;
    _pin = pinCom;
}

LimitSwitch::~LimitSwitch()
{
}

void LimitSwitch::init()
{
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
