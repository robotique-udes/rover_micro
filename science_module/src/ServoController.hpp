#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <Arduino.h>

#include <rover_lib2/actuators/servo.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

#include "config.hpp"

class ServoController
{
    static constexpr float CARROUSEL_STEP_INCREMENT_RAD = 90.0F * static_cast<float>(DEG_TO_RAD);
    static constexpr uint64_t LOOP_PERIOD_MS = 50ULL;

  public:
    void init()
    {
        this->_servoBeak.init();
        this->_servoCarrousel.init();

        this->_servoBeak.setPosition(1.06F);
        this->_servoCarrousel.setPosition(0.0F);
    };

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        this->_servoBeak.update();
        this->_servoCarrousel.update();
    };

    void setPosition(float position_, eServoType servoId_)
    {
        switch (servoId_)
        {
            case eServoType::BEAK:
                this->_servoBeak.setPosition(
                    std::clamp(position_, this->_beakServoConfig.minPosition, this->_beakServoConfig.maxPosition));
                break;

            case eServoType::CARROUSEL:
                this->_servoCarrousel.setPosition(
                    std::clamp(position_, this->_carrouselServoConfig.minPosition, this->_carrouselServoConfig.maxPosition));
                break;

            default:
                break;
        }
    };

    void nextPosCarrousel()
    {
        this->_currentCarrouselPosition += CARROUSEL_STEP_INCREMENT_RAD;

        if (this->_currentCarrouselPosition >= this->_carrouselServoConfig.maxPosition)
        {
            this->_currentCarrouselPosition = 0.0F;
        }

        this->setPosition(this->_currentCarrouselPosition, eServoType::CARROUSEL);
    }

  private:
    static constexpr Actuators::ServoT::sTimingConfig _beakServoConfig = GET_SERVO_TIMING_CONFIG<eServoType::BEAK>();
    static constexpr Actuators::ServoT::sTimingConfig _carrouselServoConfig = GET_SERVO_TIMING_CONFIG<eServoType::CARROUSEL>();

    float _currentCarrouselPosition = 0.0F;

    PWMGenerators::MCPWMTimer __pwmGenTimer
        = PWMGenerators::MCPWMTimer(_beakServoConfig.frequency, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM __beakServoPwmGen = PWMGenerators::MCPWM(PIN_SERVO_0,
                                                                  __pwmGenTimer,
                                                                  PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                                  PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    Actuators::Servo<PWMGenerators::MCPWM> _servoBeak
        = Actuators::Servo<PWMGenerators::MCPWM>(_beakServoConfig,
                                                 __beakServoPwmGen,
                                                 true,
                                                 static_cast<float>(DEG_TO_RAD) * 180.0F);

    PWMGenerators::MCPWM __carrouselServoPwmGen = PWMGenerators::MCPWM(PIN_SERVO_1,
                                                                       __pwmGenTimer,
                                                                       PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                                       PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    Actuators::Servo<PWMGenerators::MCPWM> _servoCarrousel
        = Actuators::Servo<PWMGenerators::MCPWM>(_carrouselServoConfig,
                                                 __carrouselServoPwmGen,
                                                 true,
                                                 static_cast<float>(DEG_TO_RAD) * 180.0F);

    LoopTimer<uint64_t, &Time::millis> _loopTimer = {LOOP_PERIOD_MS};
};

#endif  // SERVO_CONTROLLER_HPP