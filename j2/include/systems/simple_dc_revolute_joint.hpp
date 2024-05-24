#ifndef __SIMPLE_DC_REVOLUTE_JOINT_HPP__
#define __SIMPLE_DC_REVOLUTE_JOINT_HPP__

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "sensors/encoders/encoder.hpp"
#include "systems/joint.hpp"

#include "rover_helpers/assert.hpp"

class SimpleDcRevoluteJoint : protected Joint
{
public:
    SimpleDcRevoluteJoint(MotorDriver *DCMotor_, Encoder *encoder_, eControlMode controlMode_)
    {
        ASSERT(DCMotor_ == NULL);
        _DCMotor = DCMotor_;
        ASSERT(encoder_ == NULL);
        _encoder = encoder_;

        ASSERT(controlMode_ == eControlMode::SPEED, "Not implemented yet");
        _controlMode = controlMode_;
    }

    virtual ~SimpleDcRevoluteJoint(void) {}

    void init(void)
    {
        LOG(DEBUG, "SimpleDcRevoluteJoint init...");
        _encoder->init();
        _DCMotor->init();

        LOG(DEBUG, "SimpleDcRevoluteJoint init successful");
    }

    void update(void)
    {
        
    }

    void setPosition(float goalPosition_)
    {
        if (_controlMode != eControlMode::POSITION)
        {
            LOG(WARN, "\"setPosition\" is only available in position control mode, current control mode is: %u. No action done", (uint8_t)_controlMode);
            return;
        }

        _goalPositon = goalPosition_;
    }

    float getPosition(void)
    {
        if (_controlMode != eControlMode::POSITION)
        {
            LOG(ERROR, "\"getPosition\" is only available in position control mode, current control mode is: %u. Value will be garbage", (uint8_t)_controlMode);
        }

        _encoder->getPosition();
    }

    void setSpeed(float goalSpeed);
    float getSpeed(void);

    void calib(float calibPosition);

private:
    MotorDriver *_DCMotor = NULL;
    Encoder *_encoder = NULL;
    eControlMode _controlMode;

    float _goalPositon = 0.0f;
};

#endif // __SIMPLE_REVOLUTE_JOINT_HPP__
