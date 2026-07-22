#ifndef J5_ACTUATOR_HPP
#define J5_ACTUATOR_HPP

#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "config.hpp"
#include "MorsePlayer.hpp"

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/morse_input.hpp"

#include <cstring>

DEFINE_LOG_NODE(J5Device, Logger::eNodeState::ON);
class J5Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 500ULL;
    static constexpr float MAX_SPEED_RAD_S = 1.0F;
    static constexpr float MIN_SPEED_RAD_S = -1.0F;
    static constexpr float MIN_CMD_OPEN_LOOP = MotorDrivers::MIN_CMD_OPEN_LOOP * 0.1F;
    static constexpr float MAX_CMD_OPEN_LOOP = MotorDrivers::MAX_CMD_OPEN_LOOP * 0.1F;

    static constexpr float CAN_SEND_FREQUENCY = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(ROUND(1'000.0F / CAN_SEND_FREQUENCY));
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQUENCY * 2.0F);

    // --- Morse code message reassembly ---
    static constexpr size_t MORSE_MAX_LEN = 64;
    static constexpr uint64_t MORSE_FRAME_TIMEOUT_MS = 500ULL;

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>,
                                              RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseInput, J5Device>,
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

        // Abandon a stalled partial morse message
        if (_morseInProgress && (Time::millis() - _morseLastFrameTime > MORSE_FRAME_TIMEOUT_MS))
        {
            LOG_WARN(Logger::Nodes::J5Device, "Morse message timed out, discarding partial buffer");
            resetMorseState();
        }

        _driver.update();
        _morsePlayer.update();

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
            float cmd = MAP(targetSpeed_, MIN_SPEED_RAD_S, MAX_SPEED_RAD_S, MIN_CMD_OPEN_LOOP, MAX_CMD_OPEN_LOOP);
            cmd = std::clamp(cmd, MIN_CMD_OPEN_LOOP, MAX_CMD_OPEN_LOOP);
            _driver.setCmd(cmd);
        }
        else
        {
            _driver.setCmd(0.0F);
        }

        //_solenoid.write(_morsePlayer.isActuatorOn() ? IO::eIOState::HIGH_ : IO::eIOState::LOW_);
        _solenoid.write(IO::eIOState::LOW_);

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

    void CB_morseCodeStream(const RoverCan2::Msgs::MorseInput& msg_)
    {
        const auto& data = msg_.getData();

        if (data.start)
        {
            // New message begins — reset regardless of any prior partial state
            resetMorseState();

            if (data.msg_length > MORSE_MAX_LEN)
            {
                LOG_ERROR(Logger::Nodes::J5Device,
                          "Morse message length %u exceeds buffer of %u, dropping",
                          static_cast<unsigned>(data.msg_length),
                          static_cast<unsigned>(MORSE_MAX_LEN));
                return;
            }

            _morseInProgress = true;
            _morseExpectedLength = data.msg_length;
        }

        if (!_morseInProgress)
        {
            LOG_WARN(Logger::Nodes::J5Device, "Morse frame received with no message in progress, ignoring");
            return;
        }

        if (data.index != _morseExpectedIndex)
        {
            LOG_ERROR(Logger::Nodes::J5Device,
                      "Morse frame loss: expected index %u, got %u",
                      static_cast<unsigned>(_morseExpectedIndex),
                      static_cast<unsigned>(data.index));
            resetMorseState();
            return;
        }

        _morseBuffer[data.index] = data.character;
        _morseRunningChecksum = static_cast<uint8_t>(_morseRunningChecksum + data.character);
        _morseLastFrameTime = Time::millis();

        if (_morseRunningChecksum != data.checksum)
        {
            LOG_ERROR(Logger::Nodes::J5Device, "Morse checksum mismatch at index %u, discarding message", static_cast<unsigned>(data.index));
            resetMorseState();
            return;
        }

        ++_morseExpectedIndex;

        if (_morseExpectedIndex == _morseExpectedLength)
        {
            handleMorseComplete(_morseBuffer, _morseExpectedLength);
            resetMorseState();
        }
    }

    void handleMorseComplete(const uint8_t* buffer_, uint8_t length_)
    {
        // buffer_ is NOT null-terminated by itself; build a local terminated copy for logging/use
        char text[MORSE_MAX_LEN + 1];
        std::memcpy(text, buffer_, length_);
        text[length_] = '\0';
        Serial.printf("Morse message received (%u chars): %s\n", static_cast<unsigned>(length_), text);

        _morsePlayer.start(buffer_, length_);
    }

    void resetMorseState()
    {
        _morseInProgress = false;
        _morseExpectedIndex = 0;
        _morseExpectedLength = 0;
        _morseRunningChecksum = 0;
    }

    JointCanDeviceT _canDevice = JointCanDeviceT(
        RoverCan2::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER,
        RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J5Device>(*this, &J5Device::CB_canCmd),
        RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseInput, J5Device>(*this, &J5Device::CB_morseCodeStream),
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

    // Morse code reassembly state
    uint8_t _morseBuffer[MORSE_MAX_LEN] = {};
    uint8_t _morseExpectedIndex = 0;
    uint8_t _morseExpectedLength = 0;
    uint8_t _morseRunningChecksum = 0;
    bool _morseInProgress = false;
    uint64_t _morseLastFrameTime = 0;

    MorsePlayer _morsePlayer;
    IO::DigitalOutput _solenoid = IO::DigitalOutput(PIN_USER_LED);

    // INA219 _currentSensor = INA219(Wire, 0x85, 0.05F, 4.0F);
};

#endif  // J5ACTUATOR_HPP