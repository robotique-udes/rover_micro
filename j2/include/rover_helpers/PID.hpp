#ifndef __PID_HPP__
#define __PID_HPP__

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

class PID
{
public:
    PID(float kp_, float ki_, float kd_, float intergalLimit_);
    ~PID() {}
    void setGains(float kp_, float ki_, float kd_);
    void setIntLimit(float limit_);
    float computeCommand(float error);
    void reset();

private:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float _integralError = 0.0f;
    float _integralLimit = 0.0f;
    float _previousError = 0.0f;
    float _lastMeasureTime = 0.0f;
};

PID::PID(float kp_, float ki_, float kd_, float intergalLimit_)
{
    this->setGains(kp_, ki_, kd_);
    this->setIntLimit(intergalLimit_);
    _lastMeasureTime = micros();
}

void PID::setGains(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::setIntLimit(float limit_)
{
    _integralLimit = limit_;
}

float PID::computeCommand(float error)
{
    if (isnan(error))
    {
        error = 0.0f;
    }
    if (isinf(error))
    {
        error = 0.0f;
    }

    float currentTime = micros();
    float dt = currentTime - _lastMeasureTime;

    _integralError += error;
    _integralError = constrain(_integralError, -_integralLimit, _integralLimit);

    float cmdP = kp * error;
    float cmdI = ki * _integralError;
    cmdI = constrain(cmdI, -_integralLimit, _integralLimit);
    float cmdD = kd * (error - _previousError) / (dt / 1'000'000.0f);

    if (isnan(cmdP))
    {
        cmdP = 0.0f;
    }
    if (isnan(cmdI))
    {
        cmdI = 0.0f;
    }
    if (isnan(cmdD))
    {
        cmdD = 0.0f;
    }

    _previousError = error;
    _lastMeasureTime = currentTime;

    return cmdP + cmdI + cmdD;
}

void PID::reset()
{
    _integralError = 0;
    _previousError = 0;
}

#endif // __PID_HPP__
