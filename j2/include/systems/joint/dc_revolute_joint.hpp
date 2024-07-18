#ifndef __DC_REVOLUTE_JOINT_HPP__
#define __DC_REVOLUTE_JOINT_HPP__

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "sensors/encoders/encoder.hpp"
#include "systems/joint/joint.hpp"
#include "rover_helpers/PID.hpp"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/timer.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class DcRevoluteJoint : public Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_US = 1'000ul;

    DcRevoluteJoint(MotorDriver *DCMotor_, Encoder *encoder_, eControlMode controlMode_, PID *pidPosition_ = NULL, PID *pidSpeed_ = NULL)
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
            _pidPosition->init();

            if (pidSpeed_ == NULL)
            {
                _dualPID = false;
            }
            else
            {
                _dualPID = true;
                _pidSpeed = pidSpeed_;
                _pidSpeed->init();
            }
        }
    }

    virtual ~DcRevoluteJoint() {}

    void init(void)
    {
        LOG(DEBUG, "DcRevoluteJoint init...");
        _encoder->init();
        _DCMotor->init();

        this->initDone();
        LOG(DEBUG, "DcRevoluteJoint init successful");
    }

    void updateInternal(void)
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

                if (_dualPID)
                {
                    float cmdSpeed = _pidSpeed->computeCommand(_goalSpeed - this->getSpeed());
                    cmd = constrain(cmd, -abs(cmdSpeed), abs(cmdSpeed));
                }

                // if (this->getPosition() < _jointLimitMin)
                // {
                //     cmd = constrain(cmd, 0, MotorDriver::MAX_SPEED);
                // }
                // else if (this->getPosition() > _jointLimitMax)
                // {
                //     cmd = constrain(cmd, -MotorDriver::MAX_SPEED, 0);
                // }

                _DCMotor->setCmd(cmd);
                _currentMotorCmd = cmd;
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

    /// @brief In POSITION control mode, this will limit the joint maximum speed (but not command)
    /// @param goalSpeed_ // target speed in rad/s
    void setSpeed(float goalSpeed_)
    {
        switch (_controlMode)
        {
        case (eControlMode::POSITION):
        {
            _goalSpeed = goalSpeed_;
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
        return _encoder->getSpeed();
    }

    void calib(float calibPosition_ = 0.0f)
    {
        _encoder->calib(calibPosition_);
    }

    void setJointLimits(float min_, float max_)
    {
        ASSERT(min_ > max_, "Max joint limit can't be lower than min joint limit");
        _jointLimitMin = min_;
        _jointLimitMin = max_;

        LOG(INFO, "New joint limit applied to position range: [%.2f ; %.2f]", _jointLimitMin, _jointLimitMax);
    }

    void printDebugInfo()
    {
        LOG(DEBUG, "Cmd: %f | Position: %f | Goal: %f | Speed: %f", _DCMotor->getCmd(), _encoder->getPosition(), _goalPosition, _encoder->getSpeed());
    }

private:
    MotorDriver *_DCMotor = NULL;
    Encoder *_encoder = NULL;
    eControlMode _controlMode;
    PID *_pidPosition = NULL;
    RoverHelpers::Timer<unsigned long, micros> _timerPidLoop =
        RoverHelpers::Timer<unsigned long, micros>(1'000'000.0f / (float)PID_LOOP_FREQ_US);
    float _goalPosition = 0.0f;
    float _jointLimitMin = 0.0f;
    float _jointLimitMax = 0.0f;
    bool _dualPID;
    float _currentMotorCmd = 0.0f;
    PID *_pidSpeed = NULL;
    float _goalSpeed = 0.0f;
};

#endif // !defined(ESP32)
#endif // _REVOLUTE_JOINT_HPP__
