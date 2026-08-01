#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <Arduino.h>

#include <rover_lib2/actuators/servo.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

#include "config.hpp"

class ServoController
{
  public:
    void init() {
        _servoBeak.init();

        constexpr Actuators::ServoT::sTimingConfig servoConfig = GET_SERVO_TIMING_CONFIG<eServoType::PAN>();
        _servoBeak.setPosition(servoConfig.alignedPosition);
    };

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }
    };

    void setPosition(float );

  private:
    void runUpdateLoop() {

    };

    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    PWMGenerators::MCPWMTimer __pwmGenTimer = PWMGenerators::MCPWMTimer(GET_SERVO_TIMING_CONFIG<eServoType::BEAK>().frequency,
                                                                        PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM __beakServoPwmGen = PWMGenerators::MCPWM(PIN_SERVO_1,
                                                                  __pwmGenTimer,
                                                                  PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                                  PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    Actuators::Servo<PWMGenerators::MCPWM> _servoBeak
        = Actuators::Servo<PWMGenerators::MCPWM>(GET_SERVO_TIMING_CONFIG<eServoType::BEAK>(),
                                                 __beakServoPwmGen,
                                                 true,
                                                 static_cast<float>(DEG_TO_RAD) * 180.0F);
};

#endif  // SERVO_CONTROLLER_HPP