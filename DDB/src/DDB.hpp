#ifndef DDB_HPP
#define DDB_HPP

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/DDB_cmd.hpp"
#include "rover_can2/msgs/DDB_status.hpp"

#include "rover_lib2/IO/digital_output.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

class DDB : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbCmd, DDB>,
                                     RoverCan2::Publisher<RoverCan2::Msgs::DdbStatus, 1UL>>
{
    static constexpr uint64_t CAN_SEND_PERIOD_MS = 500ULL;

    static constexpr gpio_num_t PIN_BANK0_CH2 = GPIO_NUM_15;
    static constexpr gpio_num_t PIN_BANK0_CH3 = GPIO_NUM_6;

    static constexpr gpio_num_t PIN_BANK1_CH0 = GPIO_NUM_4;
    static constexpr gpio_num_t PIN_BANK1_CH1 = GPIO_NUM_11;
    static constexpr gpio_num_t PIN_BANK1_CH2 = GPIO_NUM_13;
    static constexpr gpio_num_t PIN_BANK1_CH3 = GPIO_NUM_21;

  public:
    DDB():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbCmd, DDB>,
                          RoverCan2::Publisher<RoverCan2::Msgs::DdbStatus, 1UL>>(
            RoverCan2::Constant::eDeviceId::DDB_CONTROLLER,
            RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbCmd, DDB>(*this, &DDB::CB_controlMsgs),
            RoverCan2::Publisher<RoverCan2::Msgs::DdbStatus, 1UL>()),
        _sendTimer(CAN_SEND_PERIOD_MS)
    {
    }

    void init(void)
    {
        _bank0_ch2.write(IO::eIOState::HIGH_);
        _bank0_ch3.write(IO::eIOState::LOW_);

        _bank1_ch0.write(IO::eIOState::HIGH_);
        _bank1_ch1.write(IO::eIOState::HIGH_);
        _bank1_ch2.write(IO::eIOState::HIGH_);
        _bank1_ch3.write(IO::eIOState::HIGH_);
    }

    void update(void)
    {
        if (_sendTimer.isReady())
        {
            RoverCan2::Msgs::DdbStatus msg;
            this->buildDDBStatusMsg(msg);

            this->sendMsg(msg);
        }
    }

    void CB_controlMsgs(const RoverCan2::Msgs::DdbCmd& msg_)
    {
        setConstantChannelOutputFromBool(_bank0_ch2, msg_.getData().B0_CH2_onState);
        setConstantChannelOutputFromBool(_bank0_ch3, msg_.getData().B0_CH3_onState);

        setConstantChannelOutputFromBool(_bank1_ch0, msg_.getData().B1_CH0_onState);
        setConstantChannelOutputFromBool(_bank1_ch1, msg_.getData().B1_CH1_onState);
        setConstantChannelOutputFromBool(_bank1_ch2, msg_.getData().B1_CH2_onState);
        setConstantChannelOutputFromBool(_bank1_ch3, msg_.getData().B1_CH3_onState);
    }

    static void setConstantChannelOutputFromBool(IO::DigitalOutput& channelIO_, bool outputOn_)
    {
        IO::eIOState state = outputOn_ ? IO::eIOState::HIGH_ : IO::eIOState::LOW_;
        channelIO_.write(state);
    }

    static void getBoolFromChannelOutput(bool& outputOn_, const IO::DigitalOutput& channelIO_)
    {
        IO::eIOState state = channelIO_.read();
        outputOn_ = (state == IO::eIOState::HIGH_);
    }

    void buildDDBStatusMsg(RoverCan2::Msgs::DdbStatus& msg_)
    {
        this->getBoolFromChannelOutput(msg_.data().B0_CH2_onState, _bank0_ch2);
        this->getBoolFromChannelOutput(msg_.data().B0_CH3_onState, _bank0_ch3);
        this->getBoolFromChannelOutput(msg_.data().B1_CH0_onState, _bank1_ch0);
        this->getBoolFromChannelOutput(msg_.data().B1_CH1_onState, _bank1_ch1);
        this->getBoolFromChannelOutput(msg_.data().B1_CH2_onState, _bank1_ch2);
        this->getBoolFromChannelOutput(msg_.data().B1_CH3_onState, _bank1_ch3);
    }

  private:
    IO::DigitalOutput _bank0_ch2 = IO::DigitalOutput(PIN_BANK0_CH2);
    IO::DigitalOutput _bank0_ch3 = IO::DigitalOutput(PIN_BANK0_CH3);

    IO::DigitalOutput _bank1_ch0 = IO::DigitalOutput(PIN_BANK1_CH0);
    IO::DigitalOutput _bank1_ch1 = IO::DigitalOutput(PIN_BANK1_CH1);
    IO::DigitalOutput _bank1_ch2 = IO::DigitalOutput(PIN_BANK1_CH2);
    IO::DigitalOutput _bank1_ch3 = IO::DigitalOutput(PIN_BANK1_CH3);

    LoopTimer<uint64_t, Time::millis> _sendTimer;

    VALIDATE_CONCEPT(RoverObject, DDB);
};

#endif  // BANK1_HPP
