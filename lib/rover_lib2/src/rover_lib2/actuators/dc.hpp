#include "actuator.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/log.hpp"

#include "rover_lib2/actuators/motor_drivers/motor_driver.hpp"
#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/controllers/controller.h"
#include "rover_lib2/controllers/PID.hpp"

DEFINE_LOG_NODE(ActuatorDc, Logger::eNodeState::OFF);

namespace Actuators
{
    enum class eControlType
    {
        SPEED,
        POSITION,
        TORQUE,
    };

    enum class eFeedbackType
    {
        OPEN_LOOP,
        CLOSE_LOOP,
    };

    template<class DriverT,
             typename EncoderT = Encoders::BaseT,
             Controllers::Controller PositionControllerT = Controllers::PID,
             Controllers::Controller SpeedControllerT = Controllers::PID>
    class DC : public Actuator<DC<DriverT, EncoderT, PositionControllerT, SpeedControllerT>>
    {
        VALIDATE_BASE_TYPE(MotorDriverT, DriverT);
        VALIDATE_BASE_TYPE(Encoders::BaseT, EncoderT);

      public:
        DC(eControlType controlType_,
           eFeedbackType feedbackType_,
           DriverT& driver_,
           EncoderT* encoder_ = nullptr,
           PositionControllerT* controllerPos_ = nullptr,
           SpeedControllerT* controllerSpeed_ = nullptr):
            _controlType(controlType_),
            _feedbackType(feedbackType_),
            _motorDriver(driver_),
            _pEncoder(encoder_),
            _pControllerPos(controllerPos_),
            _pControllerSpeed(controllerSpeed_)
        {
            switch (_controlType)
            {
                case eControlType::SPEED:
                    switch (_feedbackType)
                    {
                        case eFeedbackType::OPEN_LOOP:
                            break;
                        case eFeedbackType::CLOSE_LOOP:
                            ASSERT_COND_MSG(_pControllerSpeed, "Can't control in position without valid position controller");
                            ASSERT_COND_MSG(_pEncoder, "Can't control in position without valid encoder");
                            _errorMode = true;
                            break;
                        default:
                            ASSERT_MSG_ARGS("Unknown control mode specified: %u", std::to_underlying(_feedbackType));
                            _errorMode = true;
                            break;
                    }
                    break;

                case eControlType::POSITION:
                    ASSERT_COND_MSG(_feedbackType == eFeedbackType::CLOSE_LOOP,
                                    "Can't control DC motor in position in open loop");
                    ASSERT_COND_MSG(_pEncoder, "Can't control in position without valid encoder");
                    ASSERT_COND_MSG(_pControllerPos, "Can't control in position without valid position controller");
                    [[fallthrough]];

                case eControlType::TORQUE:
                    [[fallthrough]];

                default:
                    ASSERT_MSG_ARGS("Specified control mode not supported: %u", std::to_underlying(_controlType));
                    _errorMode = true;
            }
        }

        void __init()
        {
            _motorDriver.init();

            if (_pEncoder)
            {
                _pEncoder->init();
            }

            if (_pControllerPos)
            {
                _pControllerPos->reset();
            }

            if (_pControllerSpeed)
            {
                _pControllerSpeed->reset();
            }
        }

        void __update()
        {
            _motorDriver.update();

            if (_pEncoder)
            {
                _pEncoder->update();
            }

            if (_errorMode)
            {
                _motorDriver.setCmd(0.0F);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    speedModeUpdate();
                    break;
                case eControlType::POSITION:
                    [[fallthrough]];
                case eControlType::TORQUE:
                    [[fallthrough]];
                default:
                    ASSERT_MSG_ARGS("Control mode \"%u\" not supported, falling in safe mode", _controlType);
                    _errorMode = true;
                    break;
            }
        }

        void _setPosition(float pos_)
        {
            _goalPos = pos_;
        }

        float _getPosition()
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::ActuatorDc, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getPosition();
        }

        void _setSpeed(float speed_)
        {
            _goalSpeed = speed_;
        }

        float _getSpeed()
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::ActuatorDc, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getSpeed();
        }

        void _setMaxSpeed(float max_speed_)
        {
            _maxSpeed = max_speed_;
        }

        void _setJointLimit(std::optional<float> min_, std::optional<float> max_)
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

      private:
        void speedModeUpdate(void)
        {
            switch (_feedbackType)
            {
                case eFeedbackType::OPEN_LOOP:
                    _motorDriver.setCmd(_goalSpeed);
                    break;

                case eFeedbackType::CLOSE_LOOP:
                    if (!_pControllerSpeed || !_pEncoder)
                    {
                        _errorMode = true;
                        return;
                    }

                    float cmd = _pControllerSpeed->computeCommand(this->getSpeed(), _goalSpeed);
                    _motorDriver.setCmd(cmd);
            }
        }

        const eControlType _controlType;
        const eFeedbackType _feedbackType;

        bool _errorMode = false;

        DriverT& _motorDriver;
        EncoderT* _pEncoder;
        PositionControllerT* _pControllerPos;
        SpeedControllerT* _pControllerSpeed;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();
    };

}  // namespace Actuators
