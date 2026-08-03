#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <Arduino.h>

#include <rover_lib2/actuators/servo.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

#include "config.hpp"

class ServoController
{
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = 69UL;

  public:
    void init()
    {
        _servoBeak.init();
        _servoCarrousel.init();

        _servoBeak.setPosition(this->_beakServoConfig.alignedPosition);
        _servoCarrousel.setPosition(this->_carrouselServoConfig.alignedPosition);
    };

    void update()
    {
        _servoBeak.update();
        _servoCarrousel.update();
    };

    void setPosition(float position_, eServoType servoId_)
    {
        switch (servoId_)
        {
            case eServoType::BEAK:
                _servoBeak.setPosition(std::clamp(position_, _beakServoConfig.minPosition, _beakServoConfig.maxPosition));
                break;

            case eServoType::CARROUSEL:
                _servoCarrousel.setPosition(
                    std::clamp(position_, _carrouselServoConfig.minPosition, _carrouselServoConfig.maxPosition));
                break;

            default:
                break;
        }
    };

  private:
    static constexpr Actuators::ServoT::sTimingConfig _beakServoConfig = GET_SERVO_TIMING_CONFIG<eServoType::BEAK>();
    static constexpr Actuators::ServoT::sTimingConfig _carrouselServoConfig = GET_SERVO_TIMING_CONFIG<eServoType::CARROUSEL>();

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
};

#endif  // SERVO_CONTROLLER_HPP