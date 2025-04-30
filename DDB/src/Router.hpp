#ifndef ROUTER_HPP
#define ROUTER_HPP

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/power_cmd.hpp"
#include "rover_can2/msgs/power_status.hpp"

class Router : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerCmd, Router>,
                                        RoverCan2::Publisher<RoverCan2::Msgs::PowerStatus, 1UL>>,
               public RoverObject<Router>
{
    static constexpr uint64_t CAN_SEND_PERIOD_MS = 500ULL;
    static constexpr gpio_num_t PIN_BANK0_CH1 = GPIO_NUM_17;

  public:
    Router():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerCmd, Router>,
                          RoverCan2::Publisher<RoverCan2::Msgs::PowerStatus, 1UL>>(
            RoverCan2::Constant::eDeviceId::DDB_CONTROLLER,
            RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerCmd, Router>(*this, &Router::CB_powerCmdMsg),
            RoverCan2::Publisher<RoverCan2::Msgs::PowerStatus, 1UL>()),
        _statusSendTimer(CAN_SEND_PERIOD_MS)
    {
    }

    void _init(void) {}

    void _update(void)
    {
        if (_statusSendTimer.isReady())
        {
            RoverCan2::Msgs::PowerStatus msg;

            if (_powerControl.read() == IO::eIOState::HIGH_)
            {
                msg.data().on_state = true;
            }
            else
            {
                msg.data().on_state = false;
            }

            this->sendMsg(msg);
        }
    }

  private:
    void CB_powerCmdMsg(const RoverCan2::Msgs::PowerCmd& msg_)
    {
        if (msg_.getData().onState == true)
        {
            _powerControl.write(IO::eIOState::HIGH_);
        }
        else
        {
            _powerControl.write(IO::eIOState::LOW_);
        }
    }

    IO::DigitalOutput _powerControl = IO::DigitalOutput(PIN_BANK0_CH1);
    LoopTimer<uint64_t, Time::millis> _statusSendTimer;
};

#endif  // ROUTER_HPP
