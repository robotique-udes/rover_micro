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
    enum class eControlMode : uint8_t
    {
        POSITION,
        SPEED
    };

protected:
    Joint() {}

public:
    virtual ~Joint(){};

    virtual void init(void) = 0;
    // Ranges should be [0.0f; TWO_PI[
    virtual void setPosition(float goalPosition_) = 0;
    virtual float getPosition(void) = 0;
    // Max speed in cas of position control
    virtual void setSpeed(float goalSpeed_) = 0;
    virtual float getSpeed(void) = 0;
    virtual void calib(float calibPosition_) = 0;
    virtual void printDebugInfo() = 0;

    void update(void);
    void setJointLimits(float min_, float max_);

protected:
    bool _inited = false;

    virtual void updateInternal(void) = 0;
    
    void initDone(void);
    bool isInited(void);
    // Assert if object isn't inited, calling this before every function is a
    // good way to make sure everything is inited cleanely before using
    void checkInit(void);
};

void Joint::update(void)
{
    this->checkInit();
    this->updateInternal();
}

void Joint::initDone(void)
{
    _inited = true;
}

bool Joint::isInited(void)
{
    return _inited;
}

void Joint::checkInit(void)
{
    ASSERT(!this->isInited());
}

#endif // !defined(ESP32)
#endif // __JOINT_HPP__
