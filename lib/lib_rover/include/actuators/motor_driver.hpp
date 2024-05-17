#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

enum eDriverType
{
    TALON_SRX,
    NONE
};

class MotorDriver
{
public:
    MotorDriver(){};
    virtual ~MotorDriver(){};

    // Init ins't required but should be implemented and called prior to using the
    // parent specific pointer
    virtual void setSpeed(float spd) = 0;
    virtual void enable(void) = 0;
    virtual void disable(void) = 0;
    virtual void reset(void) = 0;
    virtual bool isMoving(void) = 0;
};

#endif // !defined(ESP32)
#endif // __MOTOR_DRIVER_HPP__
