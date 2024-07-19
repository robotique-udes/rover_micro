#ifndef __DC_REVOLUTE_JOINT_HPP__
#define __DC_REVOLUTE_JOINT_HPP__

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "sensors/encoders/encoder.hpp"
#include "systems/joint/joint.hpp"
#include "rover_helpers/PID.hpp"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/timer.hpp"
#include "rover_helpers/moving_average.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

#warning TODO: Fix 0 to 360 --> CMD = 100.0f

class DcRevoluteJoint : public Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_US = 1'000ul;
#warning TODO: Should be parameter
    static constexpr float PID_DEADBAND_RAD = 1.5f * DEG_TO_RAD;
    static constexpr uint16_t CMD_AVG_COEFF = 10ul;

    DcRevoluteJoint(MotorDriver *DCMotor_,
                    Encoder::eEncoderType encoderType_,
                    Encoder *encoder_,
                    eControlMode controlMode_,
                    PID *pidPosition_ = NULL,
                    PID *pidSpeed_ = NULL)
    {
        ASSERT(DCMotor_ == NULL);
        _DCMotor = DCMotor_;
        ASSERT(encoder_ == NULL);
        _encoder = encoder_;

        if (encoderType_ == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
        {
            _encoderType = encoderType_;
        }
        else
        {
            LOG(ERROR, "Encoder type: %u", encoderType_);
            ASSERT(true, "Encoder type not implemented/supported yet");
        }

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
                float currentPosition = this->getPosition();

                float error = 0.0f;
                if (_encoderType == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
                {
                    error = _goalPosition - currentPosition;

                    if (error >= PI)
                    {
                        error -= TWO_PI;
                    }
                    else if (error <= -PI)
                    {
                        error += TWO_PI;
                    }
                }
                else
                {
                    ASSERT("Shouldn't fall here, implementation error");
                }

                float cmd = _pidPosition->computeCommand(error);

                if (IN_ERROR(error, PID_DEADBAND_RAD, 0.0f))
                {
                    cmd = 0.0f;
                }
                else
                {
                    float cmd = _pidPosition->computeCommand(error);
                }

                if (_dualPID)
                {
                    float cmdSpeed = _pidSpeed->computeCommand(_goalSpeed - this->getSpeed());
                    cmd = constrain(cmd, -abs(cmdSpeed), abs(cmdSpeed));
                }

                if (currentPosition > _jointLimitMax && currentPosition < _jointLimitMin)
                {
                    if (abs(_jointLimitMin - currentPosition) < abs(_jointLimitMax - currentPosition))
                    {
                        cmd = constrain(cmd, 0.0f, MotorDriver::MAX_SPEED);
                    }
                    else
                    {
                        cmd = constrain(cmd, -MotorDriver::MAX_SPEED, 0.0f);
                    }
                }
                cmd = _cmdAvg.addValue(cmd);

                _DCMotor->setCmd(_cmdAvg.addValue(cmd));
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

        if (_encoderType == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
        {
            _goalPosition = CONSTRAIN_ANGLE(goalPosition_);
        }
        else
        {
            ASSERT("Shouldn't fall here, implementation error");
        }
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

        if (_encoderType == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
        {
            _jointLimitMin = CONSTRAIN_ANGLE(min_);
            _jointLimitMax = CONSTRAIN_ANGLE(max_);
        }
        else
        {
            ASSERT("Shouldn't fall here, implementation error");
        }

        LOG(INFO, "New joint limit applied to position range: [%.2f ; %.2f]", _jointLimitMin, _jointLimitMax);
    }

    void printDebugInfo()
    {
        LOG(DEBUG, "Cmd: %f | Position: %f | Goal: %f | Speed: %f", _DCMotor->getCmd(), _encoder->getPosition(), _goalPosition, _encoder->getSpeed());
    }

private:
    MotorDriver *_DCMotor = NULL;
    Encoder *_encoder = NULL;
    Encoder::eEncoderType _encoderType;
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
    RoverHelpers::MovingAverage<float, CMD_AVG_COEFF> _cmdAvg = RoverHelpers::MovingAverage<float, CMD_AVG_COEFF>(0.0f);
};

#endif // !defined(ESP32)
#endif // _REVOLUTE_JOINT_HPP__
