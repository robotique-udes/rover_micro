#ifndef __DC_REVOLUTE_JOINT_HPP__
#define __DC_REVOLUTE_JOINT_HPP__

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "systems/joint/joint.hpp"

#include "rover_helpers/timer.hpp"
#include "rover_helpers/moving_average.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class DcRevoluteJoint : public Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_US = 1'000ul;
    static constexpr uint16_t CMD_AVG_COEFF = 10ul;

    DcRevoluteJoint(MotorDriver *DCMotor_,
                    Encoder::eEncoderType encoderType_,
                    Encoder *encoder_,
                    eControlMode controlMode_,
                    bool dualPID,
                    PID *pidPosition_ = NULL,
                    PID *pidSpeed_ = NULL);
    virtual ~DcRevoluteJoint() {}

    void init(void);
    void updateInternal(void);
    void setPosition(float goalPosition_, bool switchControlMode = false);
    float getPosition(bool raw_ = false);
    /// @brief In POSITION control mode, this will limit the joint maximum speed (but not command)
    /// @param goalSpeed_ // target speed in rad/s
    void setSpeed(float goalSpeed_);
    float getSpeed(void);
    void calib(float calibPosition_ = 0.0f);
    void printDebugInfo();

private:
    MotorDriver *_DCMotor = NULL;
    Encoder *_encoder = NULL;
    RoverHelpers::Timer<unsigned long, micros> _timerPidLoop =
        RoverHelpers::Timer<unsigned long, micros>(1'000'000.0f / (float)PID_LOOP_FREQ_US);

    float _goalPosition = 0.0f;
    float _goalSpeed = 0.0f;

    RoverHelpers::MovingAverage<float, CMD_AVG_COEFF> _cmdAvg = RoverHelpers::MovingAverage<float, CMD_AVG_COEFF>(0.0f);
};

DcRevoluteJoint::DcRevoluteJoint(MotorDriver *DCMotor_,
                                 Encoder::eEncoderType encoderType_,
                                 Encoder *encoder_,
                                 eControlMode controlMode_,
                                 bool dualPID_,
                                 PID *pidPosition_,
                                 PID *pidSpeed_) : Joint(encoderType_, controlMode_, dualPID_, pidPosition_, pidSpeed_)
{
    ASSERT(DCMotor_ == NULL);
    _DCMotor = DCMotor_;
    ASSERT(encoder_ == NULL);
    _encoder = encoder_;

    this->setControlMode(controlMode_);
}

void DcRevoluteJoint::init(void)
{
    LOG(DEBUG, "DcRevoluteJoint init...");
    _encoder->init();
    _DCMotor->init();
    _DCMotor->enable();

    if (_pidPosition != NULL)
    {
        _pidPosition->init();
    }

    if (_pidSpeed != NULL)
    {
        _pidSpeed->init();
    }

    this->initDone();
    LOG(DEBUG, "DcRevoluteJoint init successful");
}

void DcRevoluteJoint::updateInternal(void)
{
    _DCMotor->update();
    _encoder->update();

    if (_timerPidLoop.isDone())
    {
        float currentPosition = this->getPosition();
        float cmd = 0.0f;

        switch (_controlMode)
        {
        case eControlMode::POSITION:
        {
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
            else if (_encoderType == Encoder::eEncoderType::ABSOLUTE_MULTI_TURN)
            {
                error = _goalPosition - currentPosition;
            }

            if (IN_ERROR(error, DEFAULT_PID_DEADBAND_RAD_POSITION, 0.0f))
            {
                cmd = 0.0f;
            }
            else
            {
                cmd = _pidPosition->computeCommand(error);
            }

            if (_dualPID)
            {
                float cmdSpeed = abs(_pidSpeed->computeCommand(_goalSpeed - this->getSpeed()));
                cmd = constrain(cmd, -cmdSpeed, cmdSpeed);
            }
            break;
        }

        case eControlMode::SPEED:
        {
            float error = _encoder->getSpeed() - _goalSpeed;
            cmd = _pidSpeed->computeCommand(error);

            if (IN_ERROR(_goalSpeed, DEFAULT_PID_DEADBAND_RAD_POSITION, 0.0f))
            {
                cmd = 0.0f;
            }
            break;
        }

        default:
        {
            LOG(FATAL, "Selected eControlMode \"%u\" not implemented or not supported", (uint8_t)_controlMode);
            ASSERT(true);
            break;
        }
        }

        cmd = this->applyJointLimits(cmd, currentPosition);
        cmd = _cmdAvg.addValue(cmd);

        _DCMotor->setCmd(_cmdAvg.addValue(cmd));
        _currentMotorCmd = cmd;
    }
}

void DcRevoluteJoint::setPosition(float goalPosition_, bool overwriteControlMode_)
{
    if (_controlMode != eControlMode::POSITION)
    {
        if (!overwriteControlMode_)
        {
            LOG(WARN, "\"setPosition\" is only available in position control mode, current control mode is: %u. No action done", (uint8_t)_controlMode);
            return;
        }
        else
        {
            this->setControlMode(eControlMode::POSITION);
        }
    }

    if (_encoderType == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
    {
        _goalPosition = CONSTRAIN_ANGLE(goalPosition_);
    }
    else if (_encoderType == Encoder::eEncoderType::ABSOLUTE_MULTI_TURN)
    {
        _goalPosition = goalPosition_;
    }
    else
    {
        ASSERT("Not implemented");
    }
}

float DcRevoluteJoint::getPosition(bool raw_)
{
    if (raw_)
    {
        return _encoder->getPosition(true);
    }
    else
    {
        return _encoder->getPosition();
    }
}

void DcRevoluteJoint::setSpeed(float goalSpeed_)
{
    if (_controlMode == eControlMode::POSITION || _controlMode == eControlMode::SPEED)
    {
        _goalSpeed = goalSpeed_;
    }
    else
    {
        ASSERT(true, "Not implemented yet");
    }
}

float DcRevoluteJoint::getSpeed(void)
{
    return _encoder->getSpeed();
}

void DcRevoluteJoint::calib(float calibPosition_)
{
    if (_encoderType == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN ||
        _encoderType == Encoder::eEncoderType::ABSOLUTE_MULTI_TURN)
    {
        _encoder->calib(CONSTRAIN_ANGLE(calibPosition_));
    }
    else
    {
        ASSERT(true, "Not implemented");
    }
}

void DcRevoluteJoint::printDebugInfo()
{
    LOG(DEBUG, "Ctr: \"%s\" | Cmd: %f | Position: %f | Goal: %f | Speed: %f",
        (_controlMode == eControlMode::POSITION) ? "pos" : "spd",
        _DCMotor->getCmd(),
        this->getPosition(),
        (_controlMode == eControlMode::POSITION) ? _goalPosition : _goalSpeed,
        _encoder->getSpeed());
}

#endif // !defined(ESP32)
#endif // _REVOLUTE_JOINT_HPP__
