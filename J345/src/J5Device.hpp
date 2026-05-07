#ifndef J5_ACTUATOR_HPP
#define J5_ACTUATOR_HPP

#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "config.hpp"

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/arm_joint_cmd.hpp"

DEFINE_LOG_NODE(J5Device, Logger::eNodeState::ON);
class J5Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 500ULL;
    static constexpr float MAX_SPEED_RAD_S = 1.0F;
    static constexpr float MIN_SPEED_RAD_S = -1.0F;
    // This limits current spikes to the drive
    static constexpr float MIN_CMD_OPEN_LOOP = MotorDrivers::MIN_CMD_OPEN_LOOP * 0.1F;
    static constexpr float MAX_CMD_OPEN_LOOP = MotorDrivers::MAX_CMD_OPEN_LOOP * 0.1F;

    static constexpr float CAN_SEND_FREQUENCY = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(ROUND(1'000.0F / CAN_SEND_FREQUENCY));
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQUENCY * 2.0F);

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    J5Device() = default;

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
            _driver.setCmd(MAX_CMD_OPEN_LOOP);
        }
        else if (_pbClose.isClicked())
        {
            _driver.setCmd(MIN_CMD_OPEN_LOOP);
        }
        else if (_canWatchdog.isOk() && !IN_ERROR(targetSpeed_, 0.001F, 0.0F))
        {
            float cmd = MAP(targetSpeed_,
                            MIN_SPEED_RAD_S,
                            MAX_SPEED_RAD_S,
                            MIN_CMD_OPEN_LOOP,
                            MAX_CMD_OPEN_LOOP);
            cmd = std::clamp(cmd, MIN_CMD_OPEN_LOOP, MAX_CMD_OPEN_LOOP);
            _driver.setCmd(cmd);
        }
        else
        {
            _driver.setCmd(0.0F);
        }

        if (_timerCanSend.isReady())
        {
            RoverCan2::Msgs::ArmJointStatus armStatusMsg;
            armStatusMsg.data().currentPosition = 0.0F;  // No position feedback on joint yet
            armStatusMsg.data().currentSpeed = _driver.getCmd();

            _canDevice.sendMsg(armStatusMsg);
        }
    }

    JointCanDeviceT& getUnderlyingCanDevice()
    {
        return _canDevice;
    }

  private:
    void CB_canCmd(const RoverCan2::Msgs::ArmJointCmd& cmd_)
    {
        _canWatchdog.reset();
        targetSpeed_ = cmd_.getData().targetSpeed;
    }

    JointCanDeviceT _canDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>(*this, &J5Device::CB_canCmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    PWMGenerators::MCPWMTimer __pwmTimer = PWMGenerators::MCPWMTimer(1'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_1);
    PWMGenerators::MCPWM __pwmGen = PWMGenerators::MCPWM(PIN_J5_PWM, __pwmTimer);
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> _driver
        = MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>(__pwmGen, PIN_J5_DIR, false);

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};

    PushButton _pbOpen = {PIN_PB_J5_OPEN};
    PushButton _pbClose = {PIN_PB_J5_CLOSE};

    float targetSpeed_ = 0.0F;
    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};
    Watchdog<uint64_t, &Time::millis> _canWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    // INA219 _currentSensor = INA219(Wire, 0x85, 0.05F, 4.0F);
};

#endif  // J5ACTUATOR_HPP
