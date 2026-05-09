#ifndef J5_ACTUATOR_HPP
#define J5_ACTUATOR_HPP

#include <Wire.h>

#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/encoder/NE12.hpp"
#include "rover_lib2/sensors/INA219.hpp"
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
    static constexpr uint8_t I2C_SLAVE_ADDRESS = static_cast<uint8_t>(0x45);
    // static constexpr uint8_t I2C_SLAVE_ADDRESS = static_cast<uint8_t>(0x85);
    static constexpr float SHUNT_RESISTANCE = 0.05F;  // Ohms
    static constexpr float ENCODER_COUNT_PER_REVOLUTION = 572.0F;

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    J5Device() = default;

    void init()
    {
        _driver.init();
        _driver.setEnabled(true);
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        _currentSensor.init();
        _encoder.init();
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
            _encoder.setDirection(cmd < 0 ? true : false);
        }
        else
        {
            _driver.setCmd(0.0F);
        }

        // _encoder.update();
        // float position = _encoder.getPosition();
        // Serial.print("Position: ");
        // Serial.println(position);

        // float speed = _encoder.getSpeed();
        // Serial.print("Speed: ");
        // Serial.println(speed);

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

    void setEncoderDirection(bool direction_)
    {
        _encoder.setDirection(direction_);
    }

  private:
    void CB_canCmd(const RoverCan2::Msgs::ArmJointCmd& cmd_)
    {
        _canWatchdog.reset();
        targetSpeed_ = cmd_.getData().targetSpeed;
    }

    float readShuntCurrent()
    {
        float shuntVoltage = _currentSensor.getShuntVoltage() / SHUNT_RESISTANCE;
        float shuntCurrent = shuntVoltage / SHUNT_RESISTANCE;

        return shuntCurrent;
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

    Filters::LowPassEMA __filterJ5Speed = {0.02F, 0.0F};    // Copied from J1Actuator.hpp, might not be ideal
    Filters::LowPassEMA __filterJ5Position = {1.0F, 0.0F};  // Copied from J1Actuator.hpp, might not be ideal

    Encoders::NE12<Filters::LowPassEMA, Filters::LowPassEMA> _encoder
        = Encoders::NE12(PIN_J5_ENC_A, PIN_J5_ENC_B, ENCODER_COUNT_PER_REVOLUTION, __filterJ5Position, __filterJ5Speed);
    INA219 _currentSensor = INA219(Wire, I2C_SLAVE_ADDRESS, SHUNT_RESISTANCE, 4.0F);
};

#endif  // J5ACTUATOR_HPP
