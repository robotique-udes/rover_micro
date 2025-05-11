#include <Arduino.h>

#include "device_config.hpp"

#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>
#include <rover_lib2/rover_object.hpp>

#include <rover_can2/device.hpp>
#include <rover_can2/msgs/prop_speed_cmd.hpp>
#include <rover_can2/msgs/prop_speed_status.hpp>

#include <rover_lib2/actuators/motor_drivers/IFX007T.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/helpers/moving_average.hpp>
#include <rover_lib2/helpers/watchdog.hpp>

constexpr gpio_num_t PIN_EN_A = GPIO_NUM_8;
constexpr gpio_num_t PIN_EN_B = GPIO_NUM_16;

constexpr gpio_num_t PIN_IN_A = GPIO_NUM_21;
constexpr gpio_num_t PIN_IN_B = GPIO_NUM_38;

constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_2;
constexpr gpio_num_t PIN_LED_R = GPIO_NUM_3;
constexpr gpio_num_t PIN_LED_G = GPIO_NUM_9;
constexpr gpio_num_t PIN_LED_B = GPIO_NUM_10;

constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_48;

constexpr uint64_t PERIOD_SEND_PROP_MOTOR_STATUS = 1'000ULL / 20ULL;
constexpr uint64_t PERIOD_RECV_PROP_MOTOR_STATUS = 1'000ULL / 20ULL;
constexpr uint64_t PERIOD_WATCHDOG_TRIGGER = 2ULL * PERIOD_RECV_PROP_MOTOR_STATUS;

static_assert(ABS(MAX_VOLTAGE) < ABS(ALIM_VOLTAGE), "Alim tension cannot be lower than limit");
constexpr float MAX_COMMAND = 100.0F * (ABS(MAX_VOLTAGE) / ABS(ALIM_VOLTAGE));

#warning TODO: Implement watchdog on new data

class PropulsionMotor : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PropSpeedCmd, PropulsionMotor>,
                                                 RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedStatus, 1UL>>,
                        public RoverObject<PropulsionMotor>
{
    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PropSpeedCmd, PropulsionMotor>,
                                      RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedStatus, 1UL>>;

  public:
    PropulsionMotor(RoverCan2::Constant::eDeviceId id_):
        DeviceT(id_,
                RoverCan2::SubscriberMember(*this, &PropulsionMotor::CB_propMotorCmdRecv),
                RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedStatus, 1UL>()),
        _timerSend(PERIOD_SEND_PROP_MOTOR_STATUS),
        _cmdAvg(0.0F),
        _watchdog(PERIOD_WATCHDOG_TRIGGER),
        _timerLoop(PERIOD_RECV_PROP_MOTOR_STATUS)
    {
        switch (id_)
        {
            case RoverCan2::Constant::eDeviceId::FRONTLEFT_MOTOR:
                _drive.setReversed(false);
                break;
            case RoverCan2::Constant::eDeviceId::FRONTRIGHT_MOTOR:
                _drive.setReversed(true);
                break;
            case RoverCan2::Constant::eDeviceId::REARLEFT_MOTOR:
                _drive.setReversed(false);
                break;
            case RoverCan2::Constant::eDeviceId::REARRIGHT_MOTOR:
                _drive.setReversed(true);
                break;
            default:
                _drive.setReversed(false);
                break;
        }
    }

    void _init(void)
    {
        _drive.setCmd(0.0F);
        _drive.init();
        _drive.setEnabled(true);
    }

    void _update(void)
    {
        _drive.update();
        _drive.setCmd(CONSTRAIN(_cmdAvg.getAverage(), -MAX_COMMAND, MAX_COMMAND));

        if (!_watchdog.isOk())
        {
            _cmdAvg.addValue(0.0F);
        }

        if (_timerSend.isReady())
        {
            RoverCan2::Msgs::PropSpeedStatus statusMsg;
            statusMsg.data().current_speed = _cmdAvg.getAverage();
            this->sendMsg(statusMsg);
        }
    }

  private:
    void CB_propMotorCmdRecv(const RoverCan2::Msgs::PropSpeedCmd& msg_)
    {
        _watchdog.reset();
        _cmdAvg.addValue(msg_.getData().target_speed);
    }

    IO::DigitalOutput __enableA
        = IO::DigitalOutput(PIN_EN_A, IO::eIOState::LOW_, GPIO_MODE_OUTPUT, GPIO_FLOATING, GPIO_DRIVE_CAP_DEFAULT);
    IO::DigitalOutput __enableB
        = IO::DigitalOutput(PIN_EN_B, IO::eIOState::LOW_, GPIO_MODE_OUTPUT, GPIO_FLOATING, GPIO_DRIVE_CAP_DEFAULT);
    PWMGenerators::MCPWMTimer __pwmTimer = PWMGenerators::MCPWMTimer(10'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM __pwmA = PWMGenerators::MCPWM(PIN_IN_A,
                                                       __pwmTimer,
                                                       PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                       PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    PWMGenerators::MCPWM __pwmB = PWMGenerators::MCPWM(PIN_IN_B,
                                                       __pwmTimer,
                                                       PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                       PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM> _drive
        = IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM>(__enableA,
                                                              __pwmA,
                                                              __enableB,
                                                              __pwmB,
                                                              false,
                                                              MotorDriverT::eBrakeMode::COAST);

    MovingAverage<float, 100> _cmdAvg;
    LoopTimer<uint64_t, Time::millis> _timerSend;
    LoopTimer<uint64_t, Time::millis> _timerLoop;
    Watchdog<uint64_t, Time::millis> _watchdog;
};

void setup()
{
    PropulsionMotor propMotor(DEVICE_ID);
    propMotor.init();

    for (EVER)
    {
        propMotor.update();
    }
}
