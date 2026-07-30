#ifndef MORSE_DEVICE_HPP
#define MORSE_DEVICE_HPP

#include "rover_lib2/sensors/push_button.hpp"
#include "config.hpp"
#include "MorsePlayer.hpp"

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/morse_code.hpp"
#include "rover_can2/msgs/morse_status.hpp"

#include <cstring>

DEFINE_LOG_NODE(MorseDevice, Logger::eNodeState::OFF);
class MorseDevice
{
    static constexpr uint64_t LOOP_PERIOD_US = 500ULL;

    static constexpr size_t MORSE_MAX_LEN = 64;
    static constexpr uint64_t MORSE_FRAME_TIMEOUT_MS = 500ULL;
    static constexpr float CAN_SEND_FREQUENCY = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(ROUND(1'000.0F / CAN_SEND_FREQUENCY));

    using MorseCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseCode, MorseDevice>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::MorseStatus>>;

  public:
    MorseDevice() = default;

    void init()
    {
        _solenoid.write(IO::eIOState::LOW_);
        Serial.println("Morse device initialized");
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        _morsePlayer.update();

        if (_morseInProgress && (Time::millis() - _morseLastFrameTime > MORSE_FRAME_TIMEOUT_MS))
        {
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse message timed out, discarding partial buffer");
            resetMorseState();
        }

        _solenoid.write(_morsePlayer.isActuatorOn() ? IO::eIOState::HIGH_ : IO::eIOState::LOW_);

        if (_timerCanSend.isReady())
        {
            RoverCan2::Msgs::MorseStatus morseStatusMsg;
            morseStatusMsg.data().is_busy = _morsePlayer.isBusy();

            _canDevice.sendMsg(morseStatusMsg);
        }
    }

    MorseCanDeviceT& getUnderlyingCanDevice()
    {
        return _canDevice;
    }

  private:
    void CB_morseCodeStream(const RoverCan2::Msgs::MorseCode& msg_)
    {
        if (_morsePlayer.isBusy())
        {
            return;
        }

        const bool start = msg_.getData().start;
        const uint8_t length = msg_.getData().msg_length;
        const uint8_t index = msg_.getData().index;
        const uint8_t character = msg_.getData().character;
        const uint8_t checksum = msg_.getData().checksum;

        if (start)
        {
            resetMorseState();

            if (length > MORSE_MAX_LEN)
            {
                LOG_WARN(Logger::Nodes::MorseDevice,
                         "Morse message length %u exceeds buffer of %u, dropping",
                         length,
                         MORSE_MAX_LEN);
                return;
            }

            _morseInProgress = true;
            _morseExpectedLength = length;
        }

        if (!_morseInProgress)
        {
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse frame received with no message in progress, ignoring");
            return;
        }

        if (index != _morseExpectedIndex)
        {
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse frame loss: expected index %d, got %d", _morseExpectedIndex, index);
            resetMorseState();
            return;
        }

        if (index >= _morseExpectedLength || index >= MORSE_MAX_LEN)
        {
            LOG_WARN(Logger::Nodes::MorseDevice,
                     "Morse index %u out of range (len=%u), discarding message",
                     index,
                     _morseExpectedLength);
            resetMorseState();
            return;
        }

        _morseBuffer[index] = character;
        _morseRunningChecksum = static_cast<uint8_t>(_morseRunningChecksum + character);
        _morseLastFrameTime = Time::millis();

        if (_morseRunningChecksum != checksum)
        {
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse checksum mismatch at index %d, discarding message", index);
            resetMorseState();
            return;
        }

        ++_morseExpectedIndex;

        if (_morseExpectedIndex == _morseExpectedLength)
        {
            _morsePlayer.start(_morseBuffer, _morseExpectedLength);
            resetMorseState();
        }
    }

    void resetMorseState()
    {
        _morseInProgress = false;
        _morseExpectedIndex = 0;
        _morseExpectedLength = 0;
        _morseRunningChecksum = 0;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};

    MorseCanDeviceT _canDevice = MorseCanDeviceT(
        RoverCan2::Constant::eDeviceId::MORSE_CODE,
        RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseCode, MorseDevice>(*this, &MorseDevice::CB_morseCodeStream),
        RoverCan2::Publisher<RoverCan2::Msgs::MorseStatus>());

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    uint8_t _morseBuffer[MORSE_MAX_LEN] = {};
    uint8_t _morseExpectedIndex = 0;
    uint8_t _morseExpectedLength = 0;
    uint8_t _morseRunningChecksum = 0;
    bool _morseInProgress = false;
    uint64_t _morseLastFrameTime = 0;

    MorsePlayer _morsePlayer;
    IO::DigitalOutput _solenoid = IO::DigitalOutput(PIN_USER_LED);
};

#endif  // MORSE_DEVICE_HPP