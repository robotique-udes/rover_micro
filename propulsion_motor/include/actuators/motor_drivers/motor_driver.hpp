#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

class MotorDriver
{
public:
    enum class eDriverType : uint8_t
    {
        TALON_SRX,
        IFX007T,
        NONE
    };

    enum class eBrakeMode : uint8_t
    {
        BRAKE,
        COAST,
        NONE
    };

    MotorDriver(){};
    virtual ~MotorDriver(){};

    virtual void init(void) = 0;
    virtual void update(void) = 0;

    // -100.0 to 100.0 for cmd
    virtual void setCmd(float cmd) = 0;
    virtual void enable(void) = 0;
    virtual void disable(void) = 0;
    virtual void reset(void) = 0;
    virtual bool isMoving(void) = 0;
    virtual float getCmd(void) = 0;

protected:
    bool _inited = false;

    void initDone(void)
    {
        _inited = true;
    }

    bool isInited(void)
    {
        return _inited;
    }

    // Assert if object isn't inited, calling this before every function is a 
    // good way to make sure everything is inited cleanely before using
    void checkInit(void)
    {
        ASSERT(!this->isInited());
    }
};

#endif // !defined(ESP32)
#endif // __MOTOR_DRIVER_HPP__
