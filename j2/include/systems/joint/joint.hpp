#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/assert.hpp"
#include "rover_helpers/PID.hpp"

class Joint
{
public:
    static constexpr float DEFAULT_PID_DEADBAND_RAD_POSITION = 1.5f * DEG_TO_RAD;
    static constexpr float DEFAULT_PID_DEADBAND_RAD_SPEED = 0.25f * DEG_TO_RAD;

    enum class eControlMode : uint8_t
    {
        POSITION,
        SPEED
    };

protected:
    /// @brief Parent constructor
    /// @param encoderType_ eEncoderType::ABSOLUTE_SINGLE_TURN or eEncoderType::ABSOLUTE_MULTI_TURN
    /// @param controlMode_ eControlMode::POSITION or eControlMode::SPEED
    /// @param dualPID_ In position control mode, the speed goal will be used as
    /// a speed limiter for the joint
    /// @param pidPosition_ PID object for position control
    /// @param pidSpeed_ PID object for speed control mode or dualPid position
    /// control mode
    Joint(Encoder::eEncoderType encoderType_, eControlMode controlMode_, bool dualPID_, PID *pidPosition_, PID *pidSpeed_);

public:
    virtual ~Joint(){};

    /// Always call this after constructor
    virtual void init(void) = 0;
    /// Rad for revolute or meter for linear
    virtual void setPosition(float goalPosition_, bool overwriteControlMode = false) = 0;
    /// Returns position in rad or in m
    virtual float getPosition(void) = 0;
    /// Set target speed in speed mode or max speed in case of dualPID control
    virtual void setSpeed(float goalSpeed_) = 0;
    /// Returns speed in rad/s or meter/s
    virtual float getSpeed(void) = 0;
    /// Calib the joint at the specified position in
    virtual void calib(float calibPosition_) = 0;
    /// Prints information to help tune pid and other
    virtual void printDebugInfo() = 0;

    /// @brief Call in each loop
    void update(void);
    /// @brief Set object to NULL to disable jog
    void setJogButton(LimitSwitch *switchREV_, float speedREV_, LimitSwitch *switchFWD_, float speedFWD_);
    void setControlMode(Joint::eControlMode controlMode_);
    /// @brief Overwrite default PIDs deadband, deadband is the tolerable error
    /// in positon or speed on which the cmd will be set to zero to keep the
    /// actuator cooler.
    /// See Joint::DEFAULT_PID_DEADBAND_RAD_POSITION or
    /// Joint::DEFAULT_PID_DEADBAND_RAD_SPEED for default values
    void setPIDDeadband(float positionDeadband_, float speedDeadband_);
    /// Sets the joint position limits and enables them
    void setJointLimits(float min_, float max_);
    /// Only use if disableJointLimits() was called after setJointLimits()
    void enableJointLimits();
    void disableJointLimits();

protected:
    bool _inited = false;
    Encoder::eEncoderType _encoderType;
    eControlMode _controlMode;

    float _currentMotorCmd = 0.0f;

    bool _dualPID;
    PID *_pidPosition = NULL;
    PID *_pidSpeed = NULL;
    float _pidDeadBandPosition = DEFAULT_PID_DEADBAND_RAD_POSITION;
    float _pidDeadBandSpeed = DEFAULT_PID_DEADBAND_RAD_SPEED;

    bool _withJointLimits = false;
    float _jointLimitMin = 0.0f;
    float _jointLimitMax = 0.0f;

    float _speedREV = 0.0f;
    float _speedFWD = 0.0f;
    LimitSwitch *_switchFWD = NULL;
    LimitSwitch *_switchREV = NULL;

    virtual void updateInternal(void) = 0;

    void initDone(void);
    bool isInited(void);
    /// Assert if object isn't inited, calling this before every function is a
    /// good way to make sure everything is inited cleanely before using
    void checkInit(void);

    /// @brief Returns the new cmd after applying the JointLimits, it's the
    /// user's job to make sure to call setJointLimit() before
    ///  for it to work properly
    float applyJointLimits(float cmd_, float currentPosition_);
};

Joint::Joint(Encoder::eEncoderType encoderType_, eControlMode controlMode_, bool dualPID_, PID *pidPosition_, PID *pidSpeed_)
{
    if (encoderType_ == Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN)
    {
        _encoderType = encoderType_;
    }
    else
    {
        LOG(ERROR, "Encoder type: %u", (uint8_t)encoderType_);
        ASSERT(true, "Encoder type not implemented/supported yet");
    }

    _dualPID = dualPID_;

    _pidPosition = pidPosition_;
    _pidSpeed = pidSpeed_;
    this->setControlMode(controlMode_);
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

void Joint::update(void)
{
    this->checkInit();

    if (_switchFWD != NULL || _switchREV != NULL)
    {
        if (_switchFWD->isClicked() && _switchREV->isClicked())
        {
            this->setControlMode(Joint::eControlMode::SPEED);
            this->setSpeed(0.0f);
        }
        else if (_switchFWD != NULL && _switchFWD->isClicked())
        {
            this->setControlMode(Joint::eControlMode::SPEED);
            this->setSpeed(50.0f);
        }
        else if (_switchREV != NULL && _switchREV->isClicked())
        {
            this->setControlMode(Joint::eControlMode::SPEED);
            this->setSpeed(-50.0f);
        }
    }

    this->updateInternal();
}

void Joint::setJogButton(LimitSwitch *switchREV_, float speedREV_, LimitSwitch *switchFWD_, float speedFWD_)
{
    _switchREV = switchREV_;
    _switchFWD = switchFWD_;

    _speedREV = speedREV_;
    _speedFWD = speedFWD_;
}

void Joint::setControlMode(Joint::eControlMode newControlMode_)
{
    if (_controlMode == newControlMode_)
    {
        return;
    }

    _controlMode = newControlMode_;

    if (_controlMode == eControlMode::POSITION)
    {
        ASSERT(_pidPosition == NULL);
        _pidPosition->init();

        if (_dualPID)
        {
            ASSERT(_pidSpeed == NULL, "Can't use dualPID mode without speed PID");
            _pidSpeed->init();
        }
    }
    else if (_controlMode == eControlMode::SPEED)
    {
        ASSERT(_pidSpeed == NULL);
        _pidSpeed->init();
    }
    else
    {
        ASSERT(true, "Implementation error");
    }
}

void Joint::setPIDDeadband(float positionDeadband_, float speedDeadband_)
{
    _pidDeadBandPosition = abs(positionDeadband_);
    _pidDeadBandSpeed = abs(speedDeadband_);
}

void Joint::setJointLimits(float min_, float max_)
{
    ASSERT(min_ > max_, "Max joint limit can't be lower than min joint limit");

    _withJointLimits = true;

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

void Joint::disableJointLimits()
{
    _withJointLimits = false;
}

void Joint::enableJointLimits()
{
    _withJointLimits = true;
}

float Joint::applyJointLimits(float cmd_, float currentPosition_)
{
    if (_withJointLimits)
    {
        if (currentPosition_ > _jointLimitMax && currentPosition_ < _jointLimitMin)
        {
            if (abs(_jointLimitMin - currentPosition_) < abs(_jointLimitMax - currentPosition_))
            {
                cmd_ = constrain(cmd_, 0.0f, MotorDriver::MAX_SPEED);
            }
            else
            {
                cmd_ = constrain(cmd_, -MotorDriver::MAX_SPEED, 0.0f);
            }
        }
    }
    return cmd_;
};

#endif // !defined(ESP32)
#endif // __JOINT_HPP__
