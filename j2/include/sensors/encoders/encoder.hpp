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

    // Init implementation is required, for object needing arguments
    // (most objects),
    // you can assert  
    virtual void init(void) = 0;
    // Ranges should be [0.0f; 360.0f[
    virtual float getPosition(void) = 0;
    virtual float getSpeed(void) = 0;
    virtual void setZero(void) = 0;
    virtual void reset(void) = 0;
};

#endif // !defined(ESP32)
#endif // __ENCODER_HPP__
