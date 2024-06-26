#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

class Encoder
{
public:
    Encoder(){};
    virtual ~Encoder(){};

    virtual void init(void) = 0;
    virtual void update(void) = 0;
    // Ranges should be [0.0f; TWO_PI[
    virtual float getPosition(void) = 0;
    virtual float getSpeed(void) = 0;
    virtual void calib(float zeroPosition) = 0;
    virtual void reset(void) = 0;

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
#endif // __ENCODER_HPP__
