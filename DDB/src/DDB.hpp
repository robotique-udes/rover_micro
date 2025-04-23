#ifndef DDB_HPP
#define DDB_HPP

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/ddb_control.hpp"

#include "rover_lib2/IO/digital_output.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

class DDB : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, DDB>,
                                     RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>>,
            public RoverObject<DDB>
{
    static constexpr uint64_t CAN_SEND_PERIOD_MS = 500ULL;

    static constexpr size_t BANK0_CHANNEL_NB = 4ULL;
    static constexpr size_t BANK1_CHANNEL_NB = 4ULL;

    static constexpr gpio_num_t PIN_BANK0_CH0 = GPIO_NUM_8;
    static constexpr gpio_num_t PIN_BANK0_CH1 = GPIO_NUM_17;
    static constexpr gpio_num_t PIN_BANK0_CH2 = GPIO_NUM_15;
    static constexpr gpio_num_t PIN_BANK0_CH3 = GPIO_NUM_6;

    static constexpr gpio_num_t PIN_BANK1_CH0 = GPIO_NUM_4;
    static constexpr gpio_num_t PIN_BANK1_CH1 = GPIO_NUM_11;
    static constexpr gpio_num_t PIN_BANK1_CH2 = GPIO_NUM_13;
    static constexpr gpio_num_t PIN_BANK1_CH3 = GPIO_NUM_21;

  public:
    DDB():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, DDB>,
                          RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>>(
            RoverCan2::Constant::eDeviceId::DDB_CONTROLLER,
            RoverCan2::SubscriberMember<RoverCan2::Msgs::DdbControl, DDB>(*this, &DDB::CB_controlMsgs),
            RoverCan2::Publisher<RoverCan2::Msgs::DdbControl, 1UL>()),
        _sendTimer(CAN_SEND_PERIOD_MS)
    {
    }

    void _init(void)
    {
        _bank0_ch0.setDutyCycle(100.0F);
        _bank0_ch1.write(IO::eIOState::HIGH_);
        _bank0_ch2.write(IO::eIOState::HIGH_);
        _bank0_ch3.write(IO::eIOState::HIGH_);

        _bank1[0].write(IO::eIOState::HIGH_);
        _bank1[1].write(IO::eIOState::HIGH_);
        _bank1[2].write(IO::eIOState::HIGH_);
        _bank1[3].write(IO::eIOState::HIGH_);
    }

    void _update(void)
    {
        if (_sendTimer.isReady())
        {
            RoverCan2::Msgs::DdbControl msg;
            this->buildStatusMsg(msg);

            this->sendMsg(msg);
        }
    }

    void CB_controlMsgs(const RoverCan2::Msgs::DdbControl& msg_)
    {
        // _bank0_ch0.setFrequency(msg_.getData().bank0_ch0_freq);
        _bank0_ch0.setDutyCycle(msg_.getData().bank0_ch0_duty);
        setConstantChannelOutputFromFreqDuty(_bank0_ch1, msg_.getData().bank0_ch1_freq, msg_.getData().bank0_ch1_duty);
        setConstantChannelOutputFromFreqDuty(_bank0_ch2, msg_.getData().bank0_ch2_freq, msg_.getData().bank0_ch2_duty);
        setConstantChannelOutputFromFreqDuty(_bank0_ch3, msg_.getData().bank0_ch3_freq, msg_.getData().bank0_ch3_duty);

        setConstantChannelOutputFromBool(_bank1[0], msg_.getData().bank1_ch0_on);
        setConstantChannelOutputFromBool(_bank1[1], msg_.getData().bank1_ch1_on);
        setConstantChannelOutputFromBool(_bank1[2], msg_.getData().bank1_ch2_on);
        setConstantChannelOutputFromBool(_bank1[3], msg_.getData().bank1_ch3_on);
    }

    static void setConstantChannelOutputFromFreqDuty(IO::DigitalOutput& channelIO_, float freq_, float duty_)
    {
        bool on = (IN_ERROR(duty_, 0.1F, 100.0F) && IN_ERROR(freq_, 0.1F, 0.0F));
        setConstantChannelOutputFromBool(channelIO_, on);
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

    static void getFreqDutyFromConstantOutput(const IO::DigitalOutput& channelIO_, float& rFreq_, float& rDuty_)
    {
        rFreq_ = 0.0F;
        IO::eIOState state = channelIO_.read();
        if (state == IO::eIOState::HIGH_)
        {
            rDuty_ = 100.0F;
        }
        else
        {
            rDuty_ = 0.0F;
        }
    }

    void buildStatusMsg(RoverCan2::Msgs::DdbControl& msg_)
    {
        msg_.data().bank0_ch0_freq = _bank0_ch0.getFrequency();
        msg_.data().bank0_ch0_duty = _bank0_ch0.getDutyCycle();

        this->getFreqDutyFromConstantOutput(_bank0_ch1, msg_.data().bank0_ch1_freq, msg_.data().bank0_ch1_duty);
        this->getFreqDutyFromConstantOutput(_bank0_ch2, msg_.data().bank0_ch2_freq, msg_.data().bank0_ch2_duty);
        this->getFreqDutyFromConstantOutput(_bank0_ch3, msg_.data().bank0_ch3_freq, msg_.data().bank0_ch3_duty);

        this->getBoolFromChannelOutput(msg_.data().bank1_ch0_on, _bank1[0]);
        this->getBoolFromChannelOutput(msg_.data().bank1_ch1_on, _bank1[1]);
        this->getBoolFromChannelOutput(msg_.data().bank1_ch2_on, _bank1[2]);
        this->getBoolFromChannelOutput(msg_.data().bank1_ch3_on, _bank1[3]);
    }

  private:
    PWMGenerators::MCPWMTimer _hardwarePwmTimer
        = PWMGenerators::MCPWMTimer(20'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);

    PWMGenerators::MCPWM _bank0_ch0 = PWMGenerators::MCPWM(PIN_BANK0_CH0, _hardwarePwmTimer);
    IO::DigitalOutput _bank0_ch1 = IO::DigitalOutput(PIN_BANK0_CH1);
    IO::DigitalOutput _bank0_ch2 = IO::DigitalOutput(PIN_BANK0_CH2);
    IO::DigitalOutput _bank0_ch3 = IO::DigitalOutput(PIN_BANK0_CH3);

    std::array<IO::DigitalOutput, BANK1_CHANNEL_NB> _bank1 = {IO::DigitalOutput(PIN_BANK1_CH0),
                                                              IO::DigitalOutput(PIN_BANK1_CH1),
                                                              IO::DigitalOutput(PIN_BANK1_CH2),
                                                              IO::DigitalOutput(PIN_BANK1_CH3)};

    LoopTimer<uint64_t, Time::millis> _sendTimer;
};

#endif  // BANK1_HPP
