#ifndef __DIFFERENTIAL_JOINT_HPP__
#define __DIFFERENTIAL_JOINT_HPP__

#include <array>

#include "Arduino.h"
#include "actuators/motor_drivers/motor_driver.hpp"
#include "systems/joint/dc_revolute_joint.hpp"

#include "rover_helpers/timer.hpp"
#include "rover_helpers/moving_average.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class DifferentialJoint
{
public:
    DifferentialJoint(Joint *jointA_, Joint *jointB_, Joint::eControlMode controlMode_)
    {
        ASSERT(jointA_ == NULL || jointB_ == NULL, "Joints pointer aren't optionnal");
        _jointA = jointA_;
        _jointB = jointB_;

        this->setControlMode(controlMode_);
    }

    void init()
    {
        _jointA->init();
        _jointA->setPosition(_jointA->getPosition());
        _jointB->init();
        _jointB->setPosition(_jointB->getPosition());
    }

    void setPosition(float upDownPosition_, float rotPosition_)
    {
        _goalPosUpDown = upDownPosition_;
        _goalPosRot = rotPosition_;
    }

    float getPositionUpDown(void)
    {
        return _currentPosUpDown;
    }

    float getPositionRot(void)
    {
        return _currentPosRot;
    }

    void setSpeed(float goalSpeedUpDown_, float goalSpeedRot_)
    {
        _goalSpeedUpDown = goalSpeedUpDown_;
        _goalSpeedRot = goalSpeedRot_;
    }

    float getSpeedUpDown(void)
    {
        return _currentSpeedUpDown;
    }

    float getSpeedRot(void)
    {
        return _currentSpeedRot;
    }

    void setJointLimitsUpDown(float min_, float max_)
    {
        _withJointLimitUpDown = true;
        _jointlimitUpDownMin = min_;
        _jointlimitUpDownMax = max_;
    }

    void setJointLimitsRot(float min_, float max_)
    {
        _withJointLimitRot = true;
        _jointlimitRotMin = min_;
        _jointlimitRotMax = max_;
    }

    void printDebugInfo()
    {
        LOG(DEBUG, "Current UP|DOWN: %f, GOAL UP|DOWN: %f, Current ROT: %f, Goal ROT: %f",
            _currentPosUpDown,
            _goalPosUpDown,
            _currentPosRot,
            _goalPosRot)
    }

    void update(void)
    {
        _jointA->update();
        _jointB->update();

        _currentPosUpDown = (_jointA->getPosition() + _jointB->getPosition()) / 2.0f;
        _currentPosRot = (-_jointA->getPosition() + _jointB->getPosition()) / 2.0f;

        applyJointLimits(&_goalPosUpDown, &_goalPosRot);

        float deltaPosUpDown = _goalPosUpDown - _currentPosUpDown;
        float deltaPosRot = _goalPosRot - _currentPosRot;

        _jointA->setPosition(_jointA->getPosition() + deltaPosUpDown - deltaPosRot/2.0f);
        _jointB->setPosition(_jointB->getPosition() + deltaPosUpDown + deltaPosRot/2.0f);
    }

    void setControlMode(Joint::eControlMode newControlMode_)
    {
        if (_controlMode == newControlMode_)
        {
            return;
        }

        _controlMode = newControlMode_;
        if (_controlMode == Joint::eControlMode::POSITION)
        {
        }
        else
        {
            ASSERT(true, "Implementation error");
        }
    }

    void disableJointLimits(void)
    {
        _withJointLimitUpDown = false;
        _withJointLimitRot = false;
    }

    void enableJointLimits(void)
    {
        if (_jointlimitUpDownMin == 0.0f || _jointlimitUpDownMax == 0.0f)
        {
            LOG(WARN, "Warning renabling joint limits on Up-Down but some are at initial values and might have been set...");
        }

        if (_jointlimitRotMin == 0.0f || _jointlimitRotMax == 0.0f)
        {
            LOG(WARN, "Warning renabling joint limits on Rotation but some are at initial values and might have been set...");
        }
    }

    void applyJointLimits(float *newPosUpDown_, float *newPosRot_)
    {
        if (_withJointLimitUpDown)
        {
            *newPosUpDown_ = constrain(*newPosUpDown_, _jointlimitUpDownMin, _jointlimitUpDownMax);
        }

        if (_withJointLimitRot)
        {
            *newPosRot_ = constrain(*newPosRot_, _jointlimitRotMin, _jointlimitRotMax);
        }
    };

private:
    Joint *_jointA = NULL;
    Joint *_jointB = NULL;

    Joint::eControlMode _controlMode = Joint::eControlMode::POSITION;

    float _goalPosUpDown = 0.0f;
    float _goalPosRot = 0.0f;
    float _goalSpeedUpDown = 0.0f;
    float _goalSpeedRot = 0.0f;

    float _currentPosUpDown = 0.0f;
    float _currentPosRot = 0.0f;
    float _currentSpeedUpDown = 0.0f;
    float _currentSpeedRot = 0.0f;

    bool _withJointLimitUpDown = false;
    bool _withJointLimitRot = false;
    float _jointlimitUpDownMin = 0.0f;
    float _jointlimitUpDownMax = 0.0f;
    float _jointlimitRotMin = 0.0f;
    float _jointlimitRotMax = 0.0f;
};

#endif // !defined(ESP32)
#endif // __DIFFERENTIAL_JOINT_HPP__
