#ifndef J5_ACTUATOR_HPP
#define J5_ACTUATOR_HPP

#include <Wire.h>

#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/INA219.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "config.hpp"

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/arm_joint_advanced_status.hpp"

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
    static constexpr uint8_t I2C_SLAVE_ADDRESS = static_cast<uint8_t>(0x45);
    static constexpr float SHUNT_RESISTANCE = 0.05F;    // Ohms
    static constexpr float MAX_STALL_CURRENT = 20.0F;   // A
    static constexpr float NO_LOAD_CURRENT = 0.53F;     // A
    static constexpr float MIN_STALL_TORQUE = 6.7689F;  // N*m
    static constexpr float MAX_TORQUE_ON_GRIPPER = 0.30F;

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointAdvancedStatus>>;

  public:
    J5Device() = default;

    void init()
    {
        _driver.init();
        _driver.setEnabled(true);
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        _currentSensor.init();
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        _driver.update();
        float currentAmps = readShuntCurrent();
        float torque = getMotorTorque(currentAmps);

        if (_pbOpen.isClicked())
        {
            // _pbOpen is currently the one closing the pince
            if (torque < MAX_TORQUE_ON_GRIPPER)
            {
                _driver.setCmd(MAX_CMD_OPEN_LOOP);
            }
            else
            {
                _driver.setCmd(0.0F);
            }
        }
        else if (_pbClose.isClicked())
        {
            _driver.setCmd(MIN_CMD_OPEN_LOOP);
        }
        else if (_canWatchdog.isOk() && !IN_ERROR(targetSpeed_, 0.001F, 0.0F))
        {
            float cmd = MAP(targetSpeed_, MIN_SPEED_RAD_S, MAX_SPEED_RAD_S, MIN_CMD_OPEN_LOOP, MAX_CMD_OPEN_LOOP);
            cmd = std::clamp(cmd, MIN_CMD_OPEN_LOOP, MAX_CMD_OPEN_LOOP);

            if (torque < MAX_TORQUE_ON_GRIPPER)
            {
                _driver.setCmd(cmd);
            }
            else
            {
                _driver.setCmd(0.0F);
            }
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

            RoverCan2::Msgs::ArmJointAdvancedStatus armStatusAdvancedMsg;
            armStatusAdvancedMsg.data().currentMotorTemp = 0.0F;  // No motor temp yet
            armStatusAdvancedMsg.data().currentTorque = getMotorTorque(currentAmps);
            armStatusAdvancedMsg.data().currentAmperage = currentAmps;

            _canDevice.sendMsg(armStatusMsg);
            _canDevice.sendMsg(armStatusAdvancedMsg);
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

    float readShuntCurrent()
    {
        float shuntVoltage = _currentSensor.getShuntVoltage();
        float shuntCurrent = shuntVoltage / SHUNT_RESISTANCE;

        return shuntCurrent;
    }

    float getMotorTorque(float current)
    {
        return (MIN_STALL_TORQUE / MAX_STALL_CURRENT) * (current - NO_LOAD_CURRENT);
    }

    JointCanDeviceT _canDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>(*this, &J5Device::CB_canCmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>(),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointAdvancedStatus>());

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

    INA219 _currentSensor = INA219(Wire, I2C_SLAVE_ADDRESS, SHUNT_RESISTANCE, 4.0F);
};

#endif  // J5ACTUATOR_HPP