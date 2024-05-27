#ifndef __PID_HPP__
#define __PID_HPP__

#include <Arduino.h>
#include "rover_helpers/macros.hpp"

class PID
{
public:
    PID(float kp_, float ki_, float kd_, float intergalLimit_);
    ~PID() {}
    void init(void);
    void setGains(float kp_, float ki_, float kd_);
    void setIntLimit(float limit_);
    float computeCommand(float error);
    void reset();

private:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float _cmdI = 0.0f;
    float _integralLimit = 0.0f;
    float _previousError = 0.0f;
    float _lastMeasureTime = 0.0f;
};

PID::PID(float kp_, float ki_, float kd_, float intergalLimit_)
{
    this->setGains(kp_, ki_, kd_);
    this->setIntLimit(intergalLimit_);
    this->reset();
    _lastMeasureTime = micros();
}

void PID::init(void)
{
    this->reset();
}

void PID::setGains(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::setIntLimit(float limit_)
{
    LOG(ERROR, "Integral limit should always be positive, abs value will be used");
    _integralLimit = abs(limit_);
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

    _cmdI += ki*error;

    float cmdP = kp * error;
    _cmdI = constrain(_cmdI, -_integralLimit, _integralLimit);
    float cmdD = kd * (error - _previousError) / (dt / 1'000'000.0f);

    if (isnan(cmdP))
    {
        cmdP = 0.0f;
    }
    if (isnan(_cmdI))
    {
        _cmdI = 0.0f;
    }
    if (isnan(cmdD))
    {
        cmdD = 0.0f;
    }

    _previousError = error;
    _lastMeasureTime = currentTime;

    // LOG(INFO, "cmdP: %.3f + cmdI: %.3f + cmdD: %.3f = %.3f", cmdP, _cmdI, cmdD, cmdP + _cmdI + cmdD);
    // LOG(INFO, "Error: %.3f", error);

    return cmdP + _cmdI + cmdD;
}

void PID::reset()
{
    _cmdI = 0.0f;
    _previousError = 0.0f;
}

#endif // __PID_HPP__
