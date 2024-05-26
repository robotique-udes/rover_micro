#ifndef __SIMPLE_DC_REVOLUTE_JOINT_HPP__
#define __SIMPLE_DC_REVOLUTE_JOINT_HPP__

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "sensors/encoders/encoder.hpp"
#include "systems/joint.hpp"
#include "rover_helpers/PID.hpp"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/timer.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class SimpleDcRevoluteJoint : protected Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ = 1'000ul;

    SimpleDcRevoluteJoint(MotorDriver *DCMotor_, Encoder *encoder_, eControlMode controlMode_, PID *pidPosition_ = NULL, PID *pidSpeed_ = NULL)
    {
        ASSERT(DCMotor_ == NULL);
        _DCMotor = DCMotor_;
        ASSERT(encoder_ == NULL);
        _encoder = encoder_;

        ASSERT(controlMode_ == eControlMode::SPEED, "Not implemented yet");
        _controlMode = controlMode_;

        if (_controlMode == eControlMode::POSITION)
        {
            ASSERT(pidPosition_ == NULL);
            _pidPosition = pidPosition_;
            _pidPosition->reset();

            if (pidSpeed_ == NULL)
            {
                _dualPID = false;
            }
            else
            {
                _dualPID = true;
            }
            _pidSpeed = pidSpeed_;
        }
    }

    virtual ~SimpleDcRevoluteJoint() {}

    void init(void)
    {
        LOG(DEBUG, "SimpleDcRevoluteJoint init...");
        _encoder->init();
        _DCMotor->init();

        LOG(DEBUG, "SimpleDcRevoluteJoint init successful");
    }

    void update(void)
    {
        _DCMotor->update();
        _encoder->update();

        if (_timerPidLoop.isDone())
        {
            switch (_controlMode)
            {
            case eControlMode::POSITION:
            {
                float cmd = _pidPosition->computeCommand(_goalPosition - this->getPosition());
                _DCMotor->setCmd(cmd);
                break;
            }

            default:
            {
                LOG(ERROR, "Selected eControlMode \"%u\" not implemented or not supported", _controlMode);
                break;
            }
            }
        }
    }

    void setPosition(float goalPosition_)
    {
        if (_controlMode != eControlMode::POSITION)
        {
            LOG(WARN, "\"setPosition\" is only available in position control mode, current control mode is: %u. No action done", (uint8_t)_controlMode);
            return;
        }

        _goalPosition = goalPosition_;
    }

    float getPosition(void)
    {
        if (_controlMode != eControlMode::POSITION)
        {
            LOG(ERROR, "\"getPosition\" is only available in position control mode, current control mode is: %u. Value will be garbage", (uint8_t)_controlMode);
        }

        return _encoder->getPosition();
    }

#warning TODO
    /// @brief In POSITION control mode, this will limit the joint maximum speed
    /// @param goalSpeed_ // target speed in rad/s
    void setSpeed(float goalSpeed_)
    {
        switch (_controlMode)
        {
        case (eControlMode::POSITION):
        {

            break;
        }

        default:
        {
            ASSERT(true, "Not implemented yet")
            break;
        }
        }
    }
    float getSpeed(void)
    {
#warning TODO
        ASSERT(true, "Not implemented yet")
    }

    void calib(float calibPosition_)
    {
#warning TODO
        ASSERT(true, "Not implemented yet")
    }

private:
    MotorDriver *_DCMotor = NULL;
    Encoder *_encoder = NULL;
    eControlMode _controlMode;
    PID *_pidPosition = NULL;
    RoverHelpers::Timer<unsigned long, micros> _timerPidLoop = RoverHelpers::Timer<unsigned long, micros>(1'000'000.0f / (float)PID_LOOP_FREQ);
    float _goalPosition = 0.0f;

    bool _dualPID;
    PID *_pidSpeed = NULL;
    float _goalSpeed = 0.0f;
};

#endif // !defined(ESP32)
#endif // __SIMPLE_REVOLUTE_JOINT_HPP__
