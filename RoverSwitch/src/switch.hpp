#ifndef DDB_HPP
#define DDB_HPP

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/ddb_control.hpp"

#include "rover_lib2/IO/digital_output.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

class SwitchETH : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, SwitchETH>,
                                           RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>>,
                  public RoverObject<SwitchETH>
{
    static constexpr uint64_t CAN_SEND_PERIOD_MS = 500ULL;

    static constexpr size_t PORT_NB = 5;
    static constexpr gpio_num_t PIN_PORT_1_POE_EN = GPIO_NUM_6;
    static constexpr gpio_num_t PIN_PORT_2_POE_EN = GPIO_NUM_7;
    static constexpr gpio_num_t PIN_PORT_3_POE_EN = GPIO_NUM_8;
    static constexpr gpio_num_t PIN_PORT_4_POE_EN = GPIO_NUM_9;
    static constexpr gpio_num_t PIN_PORT_5_POE_EN = GPIO_NUM_10;

  public:
    SwitchETH():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, SwitchETH>,
                          RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>>(
            RoverCan2::Constant::eDeviceId::DDB_CONTROLLER,
            RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, SwitchETH>(*this, &SwitchETH::CB_controlMsgs),
            RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>()),
        _sendTimer(CAN_SEND_PERIOD_MS)
    {
    }

    void _init(void)
    {
        _port1_en.write(IO::eIOState::LOW_);
        _port2_en.write(IO::eIOState::LOW_);
        _port3_en.write(IO::eIOState::LOW_);
        _port4_en.write(IO::eIOState::HIGH_);
        _port5_en.write(IO::eIOState::HIGH_);
    }

    void _update(void)
    {
        if (_sendTimer.isReady())
        {
            // TODO
        }
    }

    void CB_controlMsgs(const RoverCan2::Msgs::DdbControl& msg_)
    {
        // TODO
    }

  private:
    LoopTimer<uint64_t, Time::millis> _sendTimer;

    IO::DigitalOutput _port1_en = IO::DigitalOutput(PIN_PORT_1_POE_EN);
    IO::DigitalOutput _port2_en = IO::DigitalOutput(PIN_PORT_2_POE_EN);
    IO::DigitalOutput _port3_en = IO::DigitalOutput(PIN_PORT_3_POE_EN);
    IO::DigitalOutput _port4_en = IO::DigitalOutput(PIN_PORT_4_POE_EN);
    IO::DigitalOutput _port5_en = IO::DigitalOutput(PIN_PORT_5_POE_EN);
};

#endif  // BANK1_HPP
