#ifndef ROVER_LIB2_ACTUATORS_AK60_6_AK60_6_HPP
#define ROVER_LIB2_ACTUATORS_AK60_6_AK60_6_HPP

#include <concepts>
#include <Stream.h>

#include <array>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

#include "AK60_6_internals.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/log_plot.hpp"

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/actuators/actuator.hpp"

#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/controllers/controller.h"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/rover_object.hpp"

DEFINE_LOG_NODE(AK106, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(AK106Plot, Logger::eNodeState::OFF);

namespace Actuators
{
    class AK60_6
    {
        static constexpr float RAD_S_TO_RPM = 9.549296596425384F;
        static constexpr float N_POLE_PAIRS = 17.0F;
        static constexpr float MOTOR_REDUCTION = 6.0F;
        static constexpr float FULL_STOP_CMD = 0.0F;
        static constexpr float KT = 0.11937f;

      public:
        enum class eControlType
        {
            SPEED,
            POSITION,
            TORQUE,
        };

      private:
        enum class eSendCmd
        {
            FRAME_HEAD = 0,
            DATA_LENGTH,
            COMMAND_SET_RPM,
            RPM_MSB,
            RPM_BYTE2,
            RPM_BYTE1,
            RPM_LSB,
            CHECKSUM_MSB,
            CHECKSUM_LSB,
            FRAME_TAIL,
            eLAST
        };

        enum class eRxState : uint8_t
        {
            WAIT_HEAD = 0,
            READ_LEN,
            READ_PAYLOAD,
            READ_CRC_MSB,
            READ_CRC_LSB,
            WAIT_TAIL,
        };

        static constexpr size_t RX_MAX_PAYLOAD = 128;         // cmd + data bytes (per length field)
        static constexpr size_t RX_MAX_BYTES_PER_POLL = 128;  // cap work per update()

        static constexpr uint8_t CMD_GET_VALUES = 0x45;
        static constexpr uint8_t CMD_POS_FEEDBACK_ENABLE = 0x4C;
        static constexpr uint8_t CMD_POS_FEEDBACK_FRAME = 0x57;

      public:
        AK60_6(eControlType controlType_, std::reference_wrapper<Stream> motorSerial_):
            _controlType(controlType_),
            _motorSerial(motorSerial_)
        {
        }

        void init() {}

        void update(void)
        {
            this->pollRx();

            if (_errorMode)
            {
                LOG_WARN(Logger::Nodes::AK106, "Fallen in error mode, motor stopped");
                this->sendRadSCmd(FULL_STOP_CMD);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    this->speedModeUpdate();
                    break;
                case eControlType::POSITION:
                    [[fallthrough]];
                case eControlType::TORQUE:
                    [[fallthrough]];
                default:
                    ASSERT_MSG_ARGS("Control mode \"%u\" not supported, falling in safe mode", _controlType);
                    _errorMode = true;
            }
        }

        void speedModeUpdate(void)
        {
            float cmd = _goalSpeed;
            if (_maxJointLimit.has_value() && this->getPosition() >= _maxJointLimit.value())
            {
                cmd = std::clamp(cmd, -_maxSpeed, 0.0F);
            }

            if (_minJointLimit.has_value() && this->getPosition() <= _minJointLimit.value())
            {
                cmd = std::clamp(cmd, 0.0F, _maxSpeed);
            }

            LOG_DEBUG(Logger::Nodes::AK106, "_goalSpeed: %f | cmd: %f", _goalSpeed, cmd);
            LOG_PLOT(Logger::Nodes::AK106Plot, _goalSpeed, cmd, this->getSpeed(), this->getPosition());

            this->sendRadSCmd(cmd);
        }

        void requestGetValuesOnce()
        {
            // Frame: HEAD, LEN(=1), PAYLOAD(=0x45), CRC16(payload), TAIL
            std::array<uint8_t, 6> frame = {0};
            frame[0] = Actuators::AK60_6_Constants::FRAME_HEAD;
            frame[1] = 0x01;
            frame[2] = CMD_GET_VALUES;
            const uint16_t crc = this->calcCheckSum(&frame[2], 1);
            frame[3] = static_cast<uint8_t>((crc >> 8) & 0xFF);
            frame[4] = static_cast<uint8_t>(crc & 0xFF);
            frame[5] = Actuators::AK60_6_Constants::FRAME_TAIL;
            _motorSerial.get().write(frame.data(), frame.size());
        }

        void enablePositionFeedback(uint8_t mode_ = 0x04)
        {
            // Frame: HEAD, LEN(=2), PAYLOAD(=0x4C mode), CRC16(payload), TAIL
            std::array<uint8_t, 7> frame = {0};
            frame[0] = Actuators::AK60_6_Constants::FRAME_HEAD;
            frame[1] = 0x02;
            frame[2] = CMD_POS_FEEDBACK_ENABLE;
            frame[3] = mode_;
            const uint16_t crc = this->calcCheckSum(&frame[2], 2);
            frame[4] = static_cast<uint8_t>((crc >> 8) & 0xFF);
            frame[5] = static_cast<uint8_t>(crc & 0xFF);
            frame[6] = Actuators::AK60_6_Constants::FRAME_TAIL;
            _motorSerial.get().write(frame.data(), frame.size());
        }

        void setPosition(float /*goalPosition_*/)
        {
            ASSERT_MSG("Not implemented");
        }

        void setSpeed(float goalSpeedRad_)
        {
            _goalSpeed = goalSpeedRad_;
        }

        float getPosition(void) const
        {
            return _telemetryPos;
        }

        float getSpeed(void) const
        {
            return _telemetrySpeedRadS;
        }

        float getMotTemp(void) const
        {
            return _motTemp;
        }

        float getTorque(void) const
        {
            return _currentQ * KT;
        }

        void setMaxSpeed(float maxSpeed_)
        {
            _maxSpeed = maxSpeed_;
        }

        void calib(float /*offset_*/)
        {
            ASSERT_MSG("Not implemented");
        }

        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
            ASSERT_MSG("Not supported until get_position is implemented");
            if (min_.has_value() && max_.has_value() && min_.value() > max_.value())
            {
                ASSERT_MSG_ARGS("Invalid joint limit passed [%.3f; %.3f], actuator going into safe mode",
                                min_.value(),
                                max_.value());
                _errorMode = true;
            }

            _minJointLimit = min_;
            _maxJointLimit = max_;
        }

      private:
        void sendRadSCmd(float rad_s_)
        {
            float rpm = radSToRPM(rad_s_);
            float elecRpm_f = radToERPM(rpm);

            constexpr float MAX_ERPM = Actuators::AK60_6_Constants::RATED_SPEED_ERPM;
            elecRpm_f = std::clamp(elecRpm_f, -MAX_ERPM, MAX_ERPM);

            int32_t eRpm = static_cast<int32_t>(ROUND(elecRpm_f));

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_HEAD)] = Actuators::AK60_6_Constants::FRAME_HEAD;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::DATA_LENGTH)]
                = sizeof(eRpm) + sizeof(Actuators::AK60_6_Constants::COMMAND_SET_RPM);
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::COMMAND_SET_RPM)] = Actuators::AK60_6_Constants::COMMAND_SET_RPM;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_MSB)] = (eRpm >> 24) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE2)] = (eRpm >> 16) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE1)] = (eRpm >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_LSB)] = eRpm & 0xFF;

            uint16_t checksum = this->calcCheckSum(_sendCmdBuffer.data() + 2, 5);

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_MSB)] = (checksum >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_LSB)] = checksum & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_TAIL)] = Actuators::AK60_6_Constants::FRAME_TAIL;

            _motorSerial.get().write(_sendCmdBuffer.data(), sizeof(_sendCmdBuffer));
        }

        void pollRx(void)
        {
            auto& s = _motorSerial.get();

            size_t consumed = 0;

            while (s.available() > 0 && consumed < RX_MAX_BYTES_PER_POLL)
            {
                const int bi = s.read();
                if (bi < 0)
                {
                    break;
                }

                ++consumed;
                const uint8_t b = static_cast<uint8_t>(bi);

                switch (_rxState)
                {
                    case eRxState::WAIT_HEAD:
                        if (b == Actuators::AK60_6_Constants::FRAME_HEAD)
                        {
                            _rxLen = 0;
                            _rxIndex = 0;
                            _rxCrc = 0;
                            _rxState = eRxState::READ_LEN;
                        }
                        break;

                    case eRxState::READ_LEN:
                        _rxLen = b;
                        if ((_rxLen == 0) || (_rxLen > RX_MAX_PAYLOAD))
                        {
                            // invalid length -> resync
                            _rxState = eRxState::WAIT_HEAD;
                        }
                        else
                        {
                            _rxIndex = 0;
                            _rxState = eRxState::READ_PAYLOAD;
                        }
                        break;

                    case eRxState::READ_PAYLOAD:
                        _rxPayload[_rxIndex++] = b;
                        if (_rxIndex >= _rxLen)
                        {
                            _rxState = eRxState::READ_CRC_MSB;
                        }
                        break;

                    case eRxState::READ_CRC_MSB:
                        _rxCrc = static_cast<uint16_t>(b) << 8;
                        _rxState = eRxState::READ_CRC_LSB;
                        break;

                    case eRxState::READ_CRC_LSB:
                        _rxCrc |= static_cast<uint16_t>(b);
                        _rxState = eRxState::WAIT_TAIL;
                        break;

                    case eRxState::WAIT_TAIL:
                        if (b == Actuators::AK60_6_Constants::FRAME_TAIL)
                        {
                            const uint16_t computed = this->calcCheckSum(_rxPayload.data(), _rxLen);
                            if (computed == _rxCrc)
                            {
                                this->handleRxFrame(_rxPayload.data(), _rxLen);
                            }
                            // else: CRC mismatch, drop frame
                        }
                        // tail mismatch -> resync regardless
                        _rxState = eRxState::WAIT_HEAD;
                        break;

                    default:
                        _rxState = eRxState::WAIT_HEAD;
                        break;
                }
            }
        }

        void handleRxFrame(const uint8_t* payload_, size_t len_)
        {
            if (len_ < 1)
            {
                return;
            }

            _lastRxCmd = payload_[0];
            _lastRxLen = len_;
            _lastRxValid = true;

            const size_t copyLen = std::min(len_, _lastRxPayload.size());
            std::copy(payload_, payload_ + copyLen, _lastRxPayload.begin());

            // Decode known replies
            switch (_lastRxCmd)
            {
                case CMD_GET_VALUES:
                {
                    // Layout per Cubemars doc:
                    // 0:cmd(0x45)
                    // 1.. : mos_temp(i16)/10, motor_temp(i16)/10, out_current(i32)/100, in_current(i32)/100,
                    //       Id(i32)/100, Iq(i32)/100, throttle(i16)/1000, motor_speed(i32),
                    //       input_voltage(i16)/10, reserved(24),
                    //       status(u8), ext_loop_pos(i32)/1000, control_id(u8),
                    //       temp_reserved(6), vd(i32)/1000, vq(i32)/1000
                    size_t i = 1;

                    if (len_ < 1 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 4 + 2)
                    {
                        break;  // too short to even contain the early fields
                    }

                    _mosTemp = static_cast<float>(readI16BE(payload_, i)) / 10.0F;
                    _motTemp = static_cast<float>(readI16BE(payload_, i)) / 10.0F;

                    _currentOut = static_cast<float>(readI32BE(payload_, i)) / 100.0F;
                    _currentIn = static_cast<float>(readI32BE(payload_, i)) / 100.0F;

                    _currentD = static_cast<float>(readI32BE(payload_, i)) / 100.0F;
                    _currentQ = static_cast<float>(readI32BE(payload_, i)) / 100.0F;

                    _throttle = static_cast<float>(readI16BE(payload_, i)) / 1000.0F;

                    const int32_t motorSpeedRaw = readI32BE(payload_, i);
                    _telemetrySpeedRadS = this->erpmToRadS(motorSpeedRaw);

                    _voltageIn = static_cast<float>(readI16BE(payload_, i)) / 10.0F;

                    // Skip reserved(24)
                    if (i + 24 > len_)
                        break;
                    i += 24;

                    // status
                    if (i + 1 > len_)
                        break;
                    _telemetryFault = payload_[i++];
                    // ext loop position
                    if (i + 4 > len_)
                        break;
                    const float extLoopPos = static_cast<float>(readI32BE(payload_, i)) / 1000.0F;

                    // control id
                    if (i + 1 > len_)
                        break;
                    _telemetryId = payload_[i++];

                    // temp reserved(6)
                    if (i + 6 > len_)
                        break;
                    i += 6;

                    // vd/vq (optional; ignore if missing)
                    if (i + 8 <= len_)
                    {
                        _voltageD = static_cast<float>(readI32BE(payload_, i)) / 1000.0F;
                        _voltageQ = static_cast<float>(readI32BE(payload_, i)) / 1000.0F;
                    }

                    _telemetryPos = extLoopPos;
                    _telemetryValid = true;

                    break;
                }

                case CMD_POS_FEEDBACK_FRAME:
                {
                    // Example: AA 05 57 <pos_i32> <crc> BB, pos = int32/1000
                    if (len_ >= 1 + 4)
                    {
                        size_t i = 1;
                        const float pos = static_cast<float>(readI32BE(payload_, i)) / 1000.0F;
                        _telemetryPos = pos;
                        _telemetryValid = true;
                    }
                    break;
                }

                default:
                    break;
            }
        }

        uint16_t calcCheckSum(unsigned char* buf_, unsigned int len_) const
        {
            uint16_t cksum = 0;
            for (size_t i = 0; i < len_; i++)
            {
                cksum = Actuators::AK60_6_Constants::CRC16_TAB[(((cksum >> 8) ^ *buf_++) & 0xFF)] ^ (cksum << 8);
            }
            return cksum;
        }

        // CubeMars command
        static int16_t readI16BE(const uint8_t* p_, size_t& i_)
        {
            const int16_t v = static_cast<int16_t>((static_cast<uint16_t>(p_[i_]) << 8) | (static_cast<uint16_t>(p_[i_ + 1])));
            i_ += 2;
            return v;
        }

        // CubeMars command
        static int32_t readI32BE(const uint8_t* p_, size_t& i_)
        {
            const int32_t v = (static_cast<int32_t>(p_[i_]) << 24) | (static_cast<int32_t>(p_[i_ + 1]) << 16)
                              | (static_cast<int32_t>(p_[i_ + 2]) << 8) | (static_cast<int32_t>(p_[i_ + 3]));
            i_ += 4;
            return v;
        }

        float radSToRPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM;
        }

        float radToERPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM * N_POLE_PAIRS * MOTOR_REDUCTION;
        }

        float erpmToRadS(int32_t erpm_) const
        {
            return static_cast<float>(erpm_) / (RAD_S_TO_RPM * N_POLE_PAIRS * MOTOR_REDUCTION);
        }

        eRxState _rxState = eRxState::WAIT_HEAD;
        uint8_t _rxLen = 0;
        uint8_t _rxIndex = 0;
        uint16_t _rxCrc = 0;
        std::array<uint8_t, RX_MAX_PAYLOAD> _rxPayload = {0};

        bool _lastRxValid = false;
        uint8_t _lastRxCmd = 0;
        size_t _lastRxLen = 0;
        std::array<uint8_t, RX_MAX_PAYLOAD> _lastRxPayload = {0};

        bool _telemetryValid = false;
        float _telemetrySpeedRadS = 0.0F;  // derived from "motor speed" field (assumed ERPM)
        float _telemetryPos = 0.0F;        // external loop pos or periodic pos feedback (int32/1000)
        uint8_t _telemetryFault = 0;
        uint8_t _telemetryId = 0;

        float _mosTemp = 0.0f;
        float _motTemp = 0.0f;
        float _currentOut = 0.0f;
        float _currentIn = 0.0f;
        float _throttle = 0.0f;
        float _voltageIn = 0.0f;
        float _currentD = 0.0f;
        float _currentQ = 0.0f;
        float _voltageD = 0.0f;
        float _voltageQ = 0.0f;

        const eControlType _controlType;
        bool _errorMode = false;

        std::array<uint8_t, TO_UNDERLYING(eSendCmd::eLAST)> _sendCmdBuffer = {0};

        std::reference_wrapper<Stream> _motorSerial;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();

        VALIDATE_CONCEPT(Actuator, AK60_6);
    };
}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_AK60_6_AK60_6_HPP
