#ifndef ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP
#define ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP

#include <concepts>
#include <Stream.h>

#include "AK10-9_internals.hpp"

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

DEFINE_LOG_NODE(AK109, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(AK109Plot, Logger::eNodeState::ON);

namespace Actuators
{
    static constexpr float RAD_S_TO_RPM = 9.549296596425384F;
    static constexpr float N_POLE_PAIRS = 17.0F;
    static constexpr float MOTOR_REDUCTION = 6.0F;
    static constexpr float FULL_STOP_SPEED = 0.0F;

    enum class eControlType
    {
        SPEED,
        POSITION,
        TORQUE,
    };

    template<Encoders::Encoder EncoderT = Encoders::None,
             Controllers::Controller PositionControllerT = Controllers::None,
             Controllers::Controller SpeedControllerT = Controllers::None>
    class AK109
    {
      public:
        AK109(eControlType controlType_,
              Stream* motorSerial_,
              EncoderT* encoder_ = nullptr,
              PositionControllerT* controllerPos_ = nullptr,
              SpeedControllerT* controllerSpeed_ = nullptr):
            _controlType(controlType_),
            _motorSerial(motorSerial_),
            _pEncoder(encoder_),
            _pPositionController(controllerPos_),
            _pSpeedController(controllerSpeed_)
        {
        }

        void init()
        {
            if (_pEncoder)
            {
                _pEncoder->init();
            }
            if (_pPositionController)
            {
                _pPositionController->reset();
            }
            if (_pSpeedController)
            {
                _pSpeedController->reset();
            }
        }

        void update(void)
        {
            if (_pEncoder)
            {
                _pEncoder->update();
            }

            if (_errorMode)
            {
                LOG_WARN(Logger::Nodes::AK109, "Fallen in error mode, motor stopped");
                this->setSpeed(FULL_STOP_SPEED);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    if (_pSpeedController)
                    {
                        this->speedModeUpdate();
                    }
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
            // TODO support open loop?

            if (!_pSpeedController || !_pEncoder)
            {
                LOG_ERROR(Logger::Nodes::AK109, "No speed controller or encoder attached, falling in error mode");
                _errorMode = true;
                return;
            }

            float cmd = _pSpeedController->computeCommand(this->getSpeed(), _goalSpeed);

            if (_maxJointLimit.has_value() && this->getPosition() >= _maxJointLimit.value())
            {
                cmd = std::clamp(cmd, std::numeric_limits<float>::lowest(), 0.0F);
            }

            if (_minJointLimit.has_value() && this->getPosition() <= _minJointLimit.value())
            {
                cmd = std::clamp(cmd, 0.0F, std::numeric_limits<float>::max());
            }

            LOG_DEBUG(Logger::Nodes::AK109, "_goalSpeed: %f | cmd: %f", _goalSpeed, cmd);

            LOG_PLOT(Logger::Nodes::AK109Plot, _goalSpeed, cmd, this->getSpeed(), this->getPosition());

            this->sendCmd(cmd);
        }

        void setPosition(float goalPosition_)
        {
            ASSERT_MSG("AK109 does not support position control");
        }

        float getPosition(void) const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::AK109, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getPosition();
        }

        void setSpeed(float goalSpeedRad_)
        {
            _goalSpeed = goalSpeedRad_;
        }

        void sendCmd(float cmd_)
        {
            float rpm = cmd_ * RAD_S_TO_RPM;
            float eRpm_f = rpm * N_POLE_PAIRS * MOTOR_REDUCTION;

            constexpr float MAX_ERPM = AK10_9::RATED_SPEED_ERPM;
            eRpm_f = std::clamp(eRpm_f, -MAX_ERPM, MAX_ERPM);

            int32_t eRpm = static_cast<int32_t>(eRpm_f);

            uint8_t buffer[10];
            buffer[0] = AK10_9::FRAME_HEAD;
            buffer[1] = 0x05;
            buffer[2] = AK10_9::COMMAND_SET_RPM;
            buffer[3] = (eRpm >> 24) & 0xFF;
            buffer[4] = (eRpm >> 16) & 0xFF;
            buffer[5] = (eRpm >> 8) & 0xFF;
            buffer[6] = eRpm & 0xFF;

            uint16_t checksum = this->calcCheckSum(buffer + 2, 5);
            buffer[7] = (checksum >> 8) & 0xFF;
            buffer[8] = checksum & 0xFF;
            buffer[9] = AK10_9::FRAME_TAIL;

            _motorSerial->write(buffer, 10);
        }

        float getSpeed(void) const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::AK109, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getSpeed();
        }

        void setMaxSpeed(float maxSpeed_)
        {
            _maxSpeed = maxSpeed_;
        }

        void calib(float offset_)
        {
            if (_pEncoder)
            {
                _pEncoder->calib(offset_);
            }
        }

        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
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

        unsigned short calcCheckSum(unsigned char* buf_, unsigned int len_) const
        {
            unsigned int i;
            unsigned short cksum = 0;
            for (i = 0; i < len_; i++)
            {
                cksum = AK10_9::CRC16_TAB[(((cksum >> 8) ^ *buf_++) & 0xFF)] ^ (cksum << 8);
            }
            return cksum;
        }

      private:
        const eControlType _controlType;

        bool _errorMode = false;

        Stream* _motorSerial;
        EncoderT* _pEncoder;
        PositionControllerT* _pPositionController;
        SpeedControllerT* _pSpeedController;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();

        VALIDATE_CONCEPT(Actuator, AK109);
    };
}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP
