#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#if !defined(ESP32)
#error CPU is not supported
#else

#include <Arduino.h>
#include "rover_helpers/macros.hpp"
#include "rover_helpers/assert.hpp"
#include "rover_helpers/soft_led_blinker.hpp"

// =============================================================================
// Implementation notes:
// =============================================================================
// When creating a new child class of MotorDriver:
//  1. Implement all abstract (virtual) method
//      - In not supported method it's your choice to either print some info
//        like "not supported for driver X" or not
//
//  2. At the end of your "void init();" implementation, make sure to call:
//      this->initDone();
// =============================================================================

class MotorDriver
{
public:
    static constexpr float MAX_SPEED = 100.0f;

    enum class eDriverType : uint8_t
    {
        TALON_SRX,
        IFX007T,
        NONE
    };

    enum class eBrakeMode : uint8_t
    {
        BRAKE,
        COAST,
        NONE
    };

protected:
    static constexpr float PROTECTION_MAX_VOLTAGE = 12.0f;

    MotorDriver(MotorDriver::eBrakeMode brakeMode_ = MotorDriver::eBrakeMode::NONE)
    {
        _brakeMode = brakeMode_;
        this->setMaxVoltage(25.2f, PROTECTION_MAX_VOLTAGE, false);
    };

public:
    virtual ~MotorDriver(void){};

    virtual void init(void) = 0;
    virtual void enable(void) = 0;
    virtual void disable(void) = 0;
    virtual void reset(void) = 0;
    virtual bool isMoving(void) = 0;
    virtual float getCmd(void) = 0;
    virtual void setBrakeMode(eBrakeMode brakeMode_) = 0;
    eBrakeMode getBrakeMode(void) { return _brakeMode; }

    // -100.0 to 100.0 for cmd
    void setCmd(float cmd_)
    {
        this->checkInit();

        cmd_ = MAP(cmd_, -MAX_SPEED, MAX_SPEED, -_protectionSpeed, _protectionSpeed);
        cmd_ = constrain(cmd_, -_protectionSpeed, _protectionSpeed);
        
        this->setCmdInternal(cmd_);
    }

    void update(void)
    {
        this->checkInit();
        this->updateLed();
        this->updateInternal();
    }

    void attachRGBLed(gpio_num_t ledR_, gpio_num_t ledG_, gpio_num_t ledB_)
    {
        this->checkInit();

        _withLed = true;

        _ledR.init(ledR_, 0.0f, 0u);
        _ledG.init(ledG_, 0.0f, 0u);
        _ledB.init(ledB_, 0.0f, 0u);
    }

    // Cap the maximum voltage sent to the motor to a specified value
    void setMaxVoltage(float alimVoltage_, float maxVoltage_, bool removeOverVoltageSecurity_ = false)
    {
        if (alimVoltage_ < 0.0f || maxVoltage_ < 0.0f)
        {
            LOG(WARN, "Wrong input parameters: can't have negative voltages. New value won't be applied...");
            return;
        }

        if (maxVoltage_ > alimVoltage_)
        {
            LOG(WARN, "Wrong input parameters: can't set higher max voltage than alim voltage. New value won't be applied...");
            return;
        }

        float newMaxVoltage = 0.0f;
        if (removeOverVoltageSecurity_)
        {
            LOG(WARN, "Removing the overvoltage security will permanently damage the motor if you don't know what you're doing");
            newMaxVoltage = maxVoltage_;
        }
        else
        {
            newMaxVoltage = constrain(maxVoltage_, 0.0f, MotorDriver::PROTECTION_MAX_VOLTAGE);
        }

        LOG(WARN, "newMax: %f | alimVoltage: %f", newMaxVoltage, alimVoltage_);
        _protectionSpeed = MAP(newMaxVoltage, 0.0f, alimVoltage_, 0.0f, 100.0f);
        LOG(INFO, "New max speed set at : %f which should correspond to approx %f V", _protectionSpeed, newMaxVoltage);
    }

    float getProtectionSpeed()
    {
        return _protectionSpeed;
    }

protected:
    bool _inited = false;
    float _protectionSpeed = 0.0f;
    bool _withLed = false;
    eBrakeMode _brakeMode = MotorDriver::eBrakeMode::NONE;

    RoverHelpers::SoftLedBlinker _ledR;
    RoverHelpers::SoftLedBlinker _ledG;
    RoverHelpers::SoftLedBlinker _ledB;

    virtual void setCmdInternal(float cmd) = 0;
    virtual void updateInternal(void) = 0;

    void initDone(void)
    {
        _inited = true;
    }

    bool isInited(void)
    {
        return _inited;
    }

    // Assert if object isn't inited, calling this before every function is a
    // good way to make sure everything is inited cleanely before using but
    // induce a lost in  performance, still we'll prefer the added security
    // over the very small performance lost
    void checkInit(void)
    {
        ASSERT(!this->isInited());
    }

private:
    // Does the job for now, but would be cleaner with hardware led instead
    void updateLed(void)
    {
        if (!_withLed)
        {
            return;
        }

        float currentCmd = this->getCmd();

        if (currentCmd > 0.0f && currentCmd < 99.9f)
        {
            _ledR.setBrightness(15.0f);
            _ledR.setBlink((uint8_t)(10.0f * currentCmd / 100.0f));
            _ledG.setBrightness(0.0f);
            _ledG.setBlink(0u);
            _ledB.setBrightness(0.0f);
            _ledB.setBlink(0u);
        }
        else if (currentCmd > 99.9f)
        {
            _ledR.setBrightness(15.0f);
            _ledR.setBlink(0u);
            _ledG.setBrightness(0.0f);
            _ledG.setBlink(0u);
            _ledB.setBrightness(0.0f);
            _ledB.setBlink(0u);
        }
        else if (currentCmd < 0.0f && currentCmd > -99.9f)
        {
            _ledR.setBrightness(0.0f);
            _ledR.setBlink(0u);
            _ledG.setBrightness(8.0f);
            _ledG.setBlink((uint8_t)(10.0f * abs(currentCmd) / 100.0f));
            _ledB.setBrightness(0.0f);
            _ledB.setBlink(0u);
        }
        else if (currentCmd < -99.9f)
        {
            _ledR.setBrightness(0.0f);
            _ledR.setBlink(0u);
            _ledG.setBrightness(8.0f);
            _ledG.setBlink(0u);
            _ledB.setBrightness(0.0f);
            _ledB.setBlink(0u);
        }

        if (IN_ERROR(currentCmd, 0.001f, 0.0f))
        {
            if (_brakeMode == eBrakeMode::COAST || _brakeMode == eBrakeMode::NONE)
            {
                _ledR.setBrightness(0.0f);
                _ledR.setBlink(0u);
                _ledG.setBrightness(0.0f);
                _ledG.setBlink(0u);
                _ledB.setBrightness(1.0f);
                _ledB.setBlink(0u);
            }
            else if (_brakeMode == eBrakeMode::BRAKE)
            {
                _ledR.setBrightness(0.0f);
                _ledR.setBlink(0u);
                _ledG.setBrightness(0.0f);
                _ledG.setBlink(0u);
                _ledB.setBrightness(1.0f);
                _ledB.setBlink(1u);
            }
        }

        _ledR.update();
        _ledG.update();
        _ledB.update();
    }
};

#endif // !defined(ESP32)
#endif // __MOTOR_DRIVER_HPP__
