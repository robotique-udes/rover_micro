#ifndef MORSE_DEVICE_HPP
#define MORSE_DEVICE_HPP

#include "rover_lib2/sensors/push_button.hpp"
#include "config.hpp"
#include "MorsePlayer.hpp"

#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/morse_code.hpp"

#include <cstring>

DEFINE_LOG_NODE(MorseDevice, Logger::eNodeState::ON);
class MorseDevice
{
    // --- Morse code message reassembly ---
    static constexpr size_t MORSE_MAX_LEN = 64;
    static constexpr uint64_t MORSE_FRAME_TIMEOUT_MS = 500ULL;

    using MorseCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseCode, MorseDevice>>;

  public:
    MorseDevice() = default;

    void init()
    {
        _solenoid.write(IO::eIOState::LOW_);
        Serial.println("Morse device initialized");
    }

    void update()
    {
        _morsePlayer.update();

        if (_morseInProgress && (Time::millis() - _morseLastFrameTime > MORSE_FRAME_TIMEOUT_MS) )
        {
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse message timed out, discarding partial buffer");
            resetMorseState();
        }


        _solenoid.write(_morsePlayer.isActuatorOn() ? IO::eIOState::HIGH_ : IO::eIOState::LOW_);
    }

    MorseCanDeviceT& getUnderlyingCanDevice()
    {
        return _morseCanDevice;
    }

  private:

    void CB_morseCodeStream(const RoverCan2::Msgs::MorseCode& msg_)
    {
        // Serial.println("Here");
        const auto& data = msg_.getData();

        if (data.start)
        {
            // New message begins — reset regardless of any prior partial state
            resetMorseState();

            if (data.msg_length > MORSE_MAX_LEN)
            {
                LOG_ERROR(Logger::Nodes::MorseDevice,
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
            LOG_WARN(Logger::Nodes::MorseDevice, "Morse frame received with no message in progress, ignoring");
            return;
        }

        if (data.index != _morseExpectedIndex)
        {
            LOG_WARN(Logger::Nodes::MorseDevice,
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
            LOG_WARN(Logger::Nodes::MorseDevice,
                     "Morse checksum mismatch at index %u, discarding message",
                     static_cast<unsigned>(data.index));
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

    MorseCanDeviceT _morseCanDevice = MorseCanDeviceT(
        RoverCan2::Constant::eDeviceId::MORSE_CODE,
        RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseCode, MorseDevice>(*this, &MorseDevice::CB_morseCodeStream));

    // Morse code reassembly state
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