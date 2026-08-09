#ifndef JL_DEVICE_HPP
#define JL_DEVICE_HPP

#include "config.hpp"
#include "JLEncoder.hpp"

#include <rover_lib2/actuators/dc.hpp>
#include <rover_lib2/motor_drivers/IFX007T.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/sensors/push_button.hpp>

#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/arm_joint_cmd.hpp>
#include <rover_can2/msgs/arm_joint_status.hpp>

DEFINE_LOG_NODE(JLDevice, Logger::eNodeState::OFF);
class JLDevice
{
    static constexpr uint32_t PWM_FREQUENCY = 1'000UL;
    static constexpr uint32_t CONTROL_LOOP_PERIOD_US = 1'000UL;
    static constexpr float JOG_SPEED = 0.04F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = -0.025F;                 // m
    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m

    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_RECV_WATCHDOG_PERIOD = static_cast<uint64_t>(2.0F * 1'000.0F / CAN_RECV_FREQ);

    static constexpr float J1_MIN_JOINT_LIMIT = -0.3F;  // m
    static constexpr float J1_MAX_JOINT_LIMIT = 0.0F;   // m
    static_assert(J1_MIN_JOINT_LIMIT <= J1_MAX_JOINT_LIMIT);

    using CanDeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>,
                                         RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, JLDevice>>;

  public:
    void init()
    {
        _actuator.init();
        _actuator.setSpeed(FULL_STOP_SPEED);
        _actuator.setJointLimit(J1_MIN_JOINT_LIMIT, J1_MAX_JOINT_LIMIT);
        __motorDriver.setMaxVoltage(ALIM_VOLTAGE, MAX_MOTOR_VOLTAGE);
    }

    void update()
    {
        if (timerControlLoop.isReady())
        {
            _actuator.update();
        }

        if (_pbCalib.isClicked())
        {
            _actuator.setSpeed(FULL_STOP_SPEED);

            constexpr uint64_t CALIB_STOP_TIME = 1'000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _actuator.update();

                if (!IN_ERROR(_actuator.getSpeed(), FULL_STOP_SPEED_ERROR_TELORANCE, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _actuator.calib(CALIB_POSITION);
        }

        if (_pbFwd.isClicked())
        {
            _actuator.setSpeed(JOG_SPEED);
        }
        else if (_pbRev.isClicked())
        {
            _actuator.setSpeed(-JOG_SPEED);
        }
        else if (_cmdWatchdog.isOk())
        {
            _actuator.setSpeed(_cmdLastMsg.getData().targetSpeed);
        }
        else
        {
            _actuator.setSpeed(FULL_STOP_SPEED);
        }

        if (_timerCanSend.isReady())
        {
            RoverCan2::Msgs::ArmJointStatus msg;
            msg.data().currentPosition = _actuator.getPosition();
            msg.data().currentSpeed = _actuator.getSpeed();

            _canDevice.sendMsg(msg);
        }
    }

    CanDeviceT& getCanDevice()
    {
        return _canDevice;
    }

  private:
    void CB_armCmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _cmdLastMsg.data() = msg_.getData();
        _cmdWatchdog.reset();
    }

    PushButton _pbFwd = PushButton(PIN_PB_FWD);
    PushButton _pbRev = PushButton(PIN_PB_REV);
    PushButton _pbCalib = PushButton(PIN_PB_CALIB);

    LoopTimer<uint64_t, &Time::micros> timerControlLoop{CONTROL_LOOP_PERIOD_US};

    CanDeviceT _canDevice = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>,
                                              RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, JLDevice>>(
        RoverCan2::Constant::eDeviceId::JL_CONTROLLER,
        RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>(),
        RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, JLDevice>(*this, &JLDevice::CB_armCmd));

    RoverCan2::Msgs::ArmJointCmd _cmdLastMsg;
    Watchdog<uint64_t, &Time::millis> _cmdWatchdog = {CAN_RECV_WATCHDOG_PERIOD};
    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    // ===========================================================================================================================
    // Actuator Config
    // ===========================================================================================================================
    PWMGenerators::MCPWMTimer __pwmGen = {PWM_FREQUENCY, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0};
    PWMGenerators::MCPWM __pwmBridgeA = {PIN_MOTOR_A_IN, __pwmGen};
    IO::DigitalOutput __bridgeAEn = IO::DigitalOutput(PIN_MOTOR_A_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);
    PWMGenerators::MCPWM __pwmBridgeB = {PIN_MOTOR_B_IN, __pwmGen};
    IO::DigitalOutput __bridgeBEn = IO::DigitalOutput(PIN_MOTOR_B_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);
    MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM> __motorDriver
        = {__bridgeAEn, __pwmBridgeA, __bridgeBEn, __pwmBridgeB, false};

    Encoders::JL __encoder;

    Controllers::PID __pidSpeed = Controllers::PID(1'500.0F, 150.0F, 15.0F, 40.0F, 25'000ULL);

    Actuators::
        DC<MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM>, Encoders::JL, Controllers::None, Controllers::PID>
            _actuator
        = {Actuators::eControlType::SPEED, Actuators::eFeedbackType::CLOSE_LOOP, __motorDriver, &__encoder, nullptr, &__pidSpeed};
};

#endif  // JL_DEVICE_HPP
