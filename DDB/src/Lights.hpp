#ifndef LIGHTS_HPP
#define LIGHTS_HPP

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/PWM_cmd.hpp"
#include "rover_can2/msgs/PWM_status.hpp"
#include "rover_can2/msgs/PWM_info.hpp"

#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

class Lights : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmCmd, Lights>,
                                        RoverCan2::Publisher<RoverCan2::Msgs::PwmStatus, 1UL>,
                                        RoverCan2::Publisher<RoverCan2::Msgs::PwmInfo, 1UL>>
{
    static constexpr gpio_num_t PIN_BANK0_CH0 = GPIO_NUM_8;

    static constexpr uint64_t PERIOD_SEND_MSG_STATUS = 1'000ULL / 2ULL;
    static constexpr uint64_t PERIOD_SEND_MSG_INFO = 1'000ULL / 1ULL;

  public:
    Lights():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmCmd, Lights>,
                          RoverCan2::Publisher<RoverCan2::Msgs::PwmStatus, 1UL>,
                          RoverCan2::Publisher<RoverCan2::Msgs::PwmInfo, 1UL>>(
            RoverCan2::Constant::eDeviceId::LIGHTS_MAIN,
            RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmCmd, Lights>(*this, &Lights::CB_pwmControl),
            RoverCan2::Publisher<RoverCan2::Msgs::PwmStatus, 1UL>(),
            RoverCan2::Publisher<RoverCan2::Msgs::PwmInfo, 1UL>()),
        __mcpwmTimer(40'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0),
        _lightSignal(PIN_BANK0_CH0, __mcpwmTimer),
        _timerStatusMsg(PERIOD_SEND_MSG_STATUS),
        _timerInfoMsg(PERIOD_SEND_MSG_INFO)
    {
    }

    void init(void)
    {
        _lightSignal.init();
        _lightSignal.setDutyCycle(0.0F);
    }

    void update(void)
    {
        _lightSignal.update();

        if (_timerStatusMsg.isReady())
        {
            RoverCan2::Msgs::PwmStatus msg;
            msg.data().dutyCycle = _lightSignal.getDutyCycle();
            msg.data().frequency = _lightSignal.getFrequency();

            this->sendMsg(msg);
        }

        if (_timerInfoMsg.isReady())
        {
            RoverCan2::Msgs::PwmInfo msg;
            msg.data().dutyCycleCtrlEn = true;
            msg.data().frequencyCtrlEn = false;

            this->sendMsg(msg);
        }
    }

  private:
    void CB_pwmControl(const RoverCan2::Msgs::PwmCmd& msg_)
    {
        float value = msg_.getData().dutyCycle;
        value = std::clamp(value, 40.0f, 100.0f);
        _lightSignal.setDutyCycle(value);
    }

    PWMGenerators::MCPWMTimer __mcpwmTimer;
    PWMGenerators::MCPWM _lightSignal;

    LoopTimer<uint64_t, Time::millis> _timerStatusMsg;
    LoopTimer<uint64_t, Time::millis> _timerInfoMsg;

    VALIDATE_CONCEPT(RoverObject, Lights);
};

#endif  // LIGHTS_HPP
