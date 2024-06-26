#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/assert.hpp"

class Joint
{
public:
    enum class eControlMode: uint8_t
    {
        POSITION,
        SPEED
    };

    Joint(){};
    virtual ~Joint(){};

    virtual void init(void) = 0;
    virtual void update(void) = 0;
    // Ranges should be [0.0f; TWO_PI[
    virtual void setPosition(float goalPosition_) = 0;
    virtual float getPosition(void) = 0;
    // rad/s
    virtual void setSpeed(float goalSpeed_) = 0;
    virtual float getSpeed(void) = 0;
    virtual void calib(float calibPosition_) = 0;

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
#endif // __JOINT_HPP__
