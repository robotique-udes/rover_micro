#ifndef ACTUATOR_SERVO_HPP
#define ACTUATOR_SERVO_HPP

#include "actuator.hpp"
#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "motor_drivers/motor_driver.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"
#include "rover_lib2/helpers/chrono.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

#include <array>

DEFINE_LOG_NODE(ActuatorServo, Logger::eNodeState::ON);

namespace Actuators
{
    class ServoT
    {
      protected:
        ServoT() = default;

      public:
        struct sTimingConfig
        {
            float frequency;
            float minMs;
            float maxMs;
            float minPosition;
            float maxPosition;
            float maxSpeed;  // Used to simulate current position during movement
        };
    };

    template<typename PwmGenerator_T>
    class Servo : public Actuator<Servo<PwmGenerator_T>>
    {
        VALIDATE_BASE_TYPE(PWMGenerators::PWMGeneratorT, PwmGenerator_T);
        static constexpr float UPDATE_FREQUENCY = 1'000.0F;
        static constexpr uint64_t UPDATE_PERIOD = 1'000ULL / static_cast<uint64_t>(UPDATE_FREQUENCY);

      public:
        Servo(ServoT::sTimingConfig servoTimings_, PwmGenerator_T& pwmGenerator_, bool reversed_ = false):
            _servoTimings(servoTimings_),
            _pwmGenerator(pwmGenerator_),
            _minPosition(_servoTimings.minPosition),
            _maxPosition(_servoTimings.maxPosition),
            _maxSpeed(_servoTimings.maxSpeed),
            _msToDutyFactor(1.0F / (1'000'000.0F / _servoTimings.frequency)),
            _updateTimer(UPDATE_PERIOD)
        {
            ASSERT_COND(IN_ERROR(_pwmGenerator.getFrequency(), 10.0F, _servoTimings.frequency));
            this->setReversed(reversed_);

            this->setJointLimit(_servoTimings.minPosition, _servoTimings.maxPosition);
        }

        void __init(void)
        {
            _pwmGenerator.init();
            _pwmGenerator.setEnabled(true);
        }

        void __update(void)
        {
            if (_updateTimer.isReady())
            {
                if (_currentPos != _goalPos)
                {
                    float enlapseS = static_cast<float>(_enlapseSinceLastUpdate.getTime()) / 1'000'000.0F;
                    float maxIncrement = enlapseS * _maxSpeed;

                    float cmd = _goalPos;
                    if ((_currentPos <= _goalPos))
                    {
                        if (_currentPos + maxIncrement >= _goalPos)
                        {
                            cmd = _goalPos;
                        }
                        else
                        {
                            cmd = _currentPos + maxIncrement;
                        }
                    }
                    else
                    {
                        if (_currentPos - maxIncrement <= _goalPos)
                        {
                            cmd = _goalPos;
                        }
                        else
                        {
                            cmd = _currentPos - maxIncrement;
                        }
                    }

                    _currentPos = cmd;
                    float minDuty = _servoTimings.minMs * _msToDutyFactor;
                    float maxDuty = _servoTimings.maxMs * _msToDutyFactor;
                    cmd = MAP(cmd, _minPosition, _maxPosition, minDuty, maxDuty);
                    _pwmGenerator.setDutyCycle(100.0F * cmd);
                }

                _enlapseSinceLastUpdate.restart();
                _pwmGenerator.update();
            }
        }

        void _setPosition(float pos_)
        {
            if (!_reversed)
            {
                _goalPos = pos_;
            }
            else
            {
                _goalPos = MAP(pos_, _minPosition, _maxPosition, _maxPosition, _minPosition);
            }
            _goalPos = CONSTRAIN(_goalPos, _minPosition, _maxPosition);
        }

        float _getPosition(void)
        {
            if (!_reversed)
            {
                return _currentPos;
            }
            else
            {
                return MAP(_currentPos, _maxPosition, _minPosition, _minPosition, _maxPosition);
            }
        }

        void _setSpeed(float speed_)
        {
            ASSERT_MSG("Not supported");
        }

        float _getSpeed(void)
        {
            if (_goalPos == _currentPos)
            {
                return 0.0F;
            }
            else if (_goalPos > _currentPos)
            {
                return -_maxSpeed;
            }
            else
            {
                return _maxSpeed;
            }
        }

        void _setMaxSpeed(float max_speed_)
        {
            _maxSpeed = max_speed_;
        }

        void _setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
            if (min_ && max_)
            {
                ASSERT_COND_MSG(min_ <= max_, "Y'a dumb biatch");

                if (min_ > max_)
                {
                    _minPosition = max_.value();
                    _maxPosition = min_.value();
                }
                else
                {
                    _minPosition = min_.value();
                    _maxPosition = max_.value();
                }
            }
            else if (min_)
            {
                _minPosition = min_.value();
                _maxPosition = std::numeric_limits<float>::infinity();
            }
            else if (max_)
            {
                _minPosition = -std::numeric_limits<float>::infinity();
                _maxPosition = max_.value();
            }
            else
            {
                _minPosition = -std::numeric_limits<float>::infinity();
                _maxPosition = std::numeric_limits<float>::infinity();
            }

            _goalPos = CONSTRAIN(_goalPos, _minPosition, _maxPosition);
        }

        void _setReversed(bool reversed_)
        {
            _reversed = reversed_;
        }

      private:
        const ServoT::sTimingConfig _servoTimings;
        PwmGenerator_T& _pwmGenerator;

        float _goalPos = 0.0F;
        float _currentPos = 0.0F;
        bool _reversed = false;

        float _minPosition;
        float _maxPosition;
        float _maxSpeed;

        const float _msToDutyFactor;
        Chrono<uint64_t, Time::micros> _enlapseSinceLastUpdate;
        LoopTimer<uint64_t, Time::millis> _updateTimer;
    };

}  // namespace Actuators

#endif  // ACTUATOR_SERVO_HPP
