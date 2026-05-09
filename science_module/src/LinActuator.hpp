#ifndef LIN_ACTUATOR_HPP
#define LIN_ACTUATOR_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/DRV8251A.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

#include <algorithm>

DEFINE_LOG_NODE(LinearAct, Logger::eNodeState::OFF);

class LinearAct
{
    static constexpr float CONTROL_LOOP_FREQUENCY_HZ = 1000.0F;
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = static_cast<uint64_t>(ROUND(1'000'000.0F / CONTROL_LOOP_FREQUENCY_HZ));
    static constexpr float MAX_MOTOR_SPEED_RAD_S = 100.0F;
    static_assert(MAX_MOTOR_SPEED_RAD_S >= 0.0F);

  public:
    void init()
    {
        _linAct.setJointLimit(std::nullopt, std::nullopt);
        _linAct.setSpeed(0.0F);
        _linAct.setMaxSpeed(MAX_MOTOR_SPEED_RAD_S);
        _linAct.init();
        __motorDriver.setMaxVoltage(ALIM_VOLTAGE, MAX_MOTOR_VOLTAGE);
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        this->runningUpdateLoop();
    }

    void setSpeed(float speed_)
    {
        _linActSpeedGoal = std::clamp(speed_, -MAX_MOTOR_SPEED_RAD_S, MAX_MOTOR_SPEED_RAD_S);
    }

    float getSpeed() const
    {
        return _linActSpeedGoal;
    }

  private:
    void runningUpdateLoop()
    {
        _linAct.update();
        float speedCmd = _linActSpeedGoal;

        _linAct.setSpeed(speedCmd);
    }

    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _linActSpeedGoal = 0.0F;

    // PWM
    PWMGenerators::MCPWMTimer __linAct_pwmGeneratorTimer = {1'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_1};

    PWMGenerators::MCPWM __pwmBridgeA = {PIN_LIN_ACT_1, __linAct_pwmGeneratorTimer};
    PWMGenerators::MCPWM __pwmBridgeB = {PIN_LIN_ACT_2, __linAct_pwmGeneratorTimer};

    MotorDrivers::DRV8251A<PWMGenerators::MCPWM, PWMGenerators::MCPWM> __motorDriver = {__pwmBridgeA, __pwmBridgeB, false};

    Actuators::DC<MotorDrivers::DRV8251A<PWMGenerators::MCPWM, PWMGenerators::MCPWM>,
                  Encoders::None,
                  Controllers::None,
                  Controllers::None>
        _linAct = {Actuators::eControlType::SPEED, Actuators::eFeedbackType::OPEN_LOOP, __motorDriver, nullptr, nullptr, nullptr};
};

#endif