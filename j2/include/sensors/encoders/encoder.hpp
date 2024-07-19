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
    enum class eEncoderType : uint8_t 
    {
        ABSOLUTE_SINGLE_TURN,
        ABSOLUTE_MULTI_TURN
    };

protected:
    Encoder(bool reversed_)
    {
        _reversed = reversed_;
    };

public:
    virtual ~Encoder(){};

    virtual void init(void) = 0;
    // Units should be rads for angular joint and meters for linear joints
    float getPosition(void)
    {
        return _reversed ? -this->getPositionInternal() : this->getPositionInternal();
    }
    float getSpeed(void)
    {
        return _reversed ? -this->getSpeedInternal() : this->getSpeedInternal();
    }
    virtual void calib(float zeroPosition) = 0;
    virtual void reset(void) = 0;

    void update(void)
    {
        this->checkInit();
        this->updateInternal();
    }

protected:
    bool _inited = false;
    bool _reversed = false;

    virtual void updateInternal(void) = 0;
    virtual float getPositionInternal(void) = 0;
    virtual float getSpeedInternal(void) = 0;

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
