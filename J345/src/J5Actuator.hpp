#ifndef J5_ACTUATOR_HPP
#define J5_ACTUATOR_HPP

#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "config.hpp"

class J5Actuator
{
    static constexpr uint64_t LOOP_PERIOD_US = 500ULL;

  public:
    J5Actuator() = default;

    void init()
    {
        _driver.init();
        _driver.setEnabled(true);
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        _driver.update();

        if (_pbOpen.isClicked())
        {
            _driver.setCmd(100.0F);
        }
        else if (_pbClose.isClicked())
        {
            _driver.setCmd(-100.0F);
        }
        else
        {
            _driver.setCmd(0.0F);
        }
    }

  private:
    PWMGenerators::MCPWMTimer __pwmTimer = PWMGenerators::MCPWMTimer(1'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_1);
    PWMGenerators::MCPWM __pwmGen = PWMGenerators::MCPWM(PIN_J5_PWM, __pwmTimer);
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> _driver
        = MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>(__pwmGen, PIN_J5_DIR, false);

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};

    PushButton _pbOpen = {PIN_PB_J5_OPEN};
    PushButton _pbClose = {PIN_PB_J5_CLOSE};
};

#endif  // J5ACTUATOR_HPP
