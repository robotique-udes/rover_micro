#ifndef ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP
#define ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP

#include <algorithm>
#include <cmath>
#include <utility>

#include "rover_lib2/motor_drivers/motor_driver.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(DRV8251A, Logger::eNodeState::OFF);

namespace MotorDrivers
{
    template<PWMGenerators::PWMGenerator PwmGenerator1T, PWMGenerators::PWMGenerator PwmGenerator2T>
    class DRV8251A
    {
        static constexpr float FULL_STOP_CMD = 0.0F;
        static constexpr float COAST_STOPPED_ERROR_TOLERANCE = 0.001F;

      public:
        DRV8251A(PwmGeneratorAT& pwmA_, PwmGeneratorBT& pwmB_, bool reversed_, eBrakeMode brakeMode_ = eBrakeMode::BRAKE):
            _pwmA(pwmA_),
            _pwmB(pwmB_),
            _goalCmd(0.0F),
            _reversed(reversed_),
            _brakeMode(brakeMode_)
        {
            this->setReversed(_reversed);
            this->setBrakeMode(brakeMode_);
        }

        void init(void)
        {
            _pwmA.init();
            _pwmB.init();

            this->setCmd(FULL_STOP_CMD);
        }

        void update(void)
        {
            switch (_brakeMode)
            {
                case eBrakeMode::BRAKE:
                {
                    // Stopped: both at 0% → active brake (both INx pulled low)
                    // Moving:  one channel gets cmd, other stays 0%
                    float cmd = std::abs(_goalCmd);
                    bool forward = (_goalCmd >= FULL_STOP_CMD);

                    if (forward)
                    {
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "BRAKE FWD | A: %f, B: %f", cmd, FULL_STOP_CMD);
                        _pwmA.setDutyCycle(cmd);
                        _pwmB.setDutyCycle(FULL_STOP_CMD);
                    }
                    else
                    {
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "BRAKE REV | A: %f, B: %f", FULL_STOP_CMD, cmd);
                        _pwmA.setDutyCycle(FULL_STOP_CMD);
                        _pwmB.setDutyCycle(cmd);
                    }
                    break;
                }
                case eBrakeMode::COAST:
                {
                    bool stopped = IN_ERROR(_goalCmd, COAST_STOPPED_ERROR_TOLERANCE, FULL_STOP_CMD);

                    if (stopped)
                    {
                        // Both INx at 100% → coast (both pulled high = Hi-Z output)
                        _pwmA.setDutyCycle(MAX_CMD_OPEN_LOOP);
                        _pwmB.setDutyCycle(MAX_CMD_OPEN_LOOP);
                    }
                    else
                    {
                        float cmd = std::abs(_goalCmd);
                        bool forward = (_goalCmd >= FULL_STOP_CMD);

                        if (forward)
                        {
                            _pwmA.setDutyCycle(cmd);
                            _pwmB.setDutyCycle(FULL_STOP_CMD);
                        }
                        else
                        {
                            _pwmA.setDutyCycle(FULL_STOP_CMD);
                            _pwmB.setDutyCycle(cmd);
                        }
                    }
                    break;
                }
                default:
                    _pwmA.setDutyCycle(FULL_STOP_CMD);
                    _pwmB.setDutyCycle(FULL_STOP_CMD);
                    _pwmA.update();
                    _pwmB.update();
                    ASSERT_MSG_ARGS("Unknown brake mode : %u", std::to_underlying(_brakeMode));
            }

            _pwmA.update();
            _pwmB.update();
        }
        // Range [-100; 100]
        void setCmd(float cmd_)
        {
            _goalCmd = CONSTRAIN(cmd_, -_maxCommand, _maxCommand);
            if (_reversed)
            {
                _goalCmd = -_goalCmd;
            }
        }

        float getCmd(void) const
        {
            return _reversed ? -_goalCmd : _goalCmd;
        }

        void setReversed(bool reversed_)
        {
            _reversed = reversed_;
            this->setCmd(this->getCmd());
        }

        bool isReversed(void) const
        {
            return _reversed;
        }

        void setBrakeMode(eBrakeMode mode_)
        {
            _brakeMode = mode_;
        }
        eBrakeMode getBrakeMode(void) const
        {
            return _brakeMode;
        }

        void setMaxCmd(float cmd_)
        {
            float newMax = std::abs(cmd_);
            newMax = std::clamp(newMax, 0.0F, MotorDrivers::MAX_CMD_OPEN_LOOP);
            _maxCommand = newMax;
            this->setCmd(this->getCmd());
        }

        void setMaxVoltage(float alim_, float maxVoltage_)
        {
            float absAlim = std::abs(alim_);
            float absMaxVoltage = std::clamp(std::abs(maxVoltage_), 0.0F, absAlim);
            this->setMaxCmd((absMaxVoltage / absAlim) * MotorDrivers::MAX_CMD_OPEN_LOOP);
        }

      private:
        PwmGeneratorAT& _pwmA;
        PwmGeneratorBT& _pwmB;

        float _maxCommand = MotorDrivers::MAX_CMD_OPEN_LOOP;
        float _goalCmd;
        bool _reversed;
        eBrakeMode _brakeMode;

        VALIDATE_CONCEPT(MotorDriver, DRV8251A);
    };

    // Deduction guide — matches constructor signature exactly
    template<typename PwmGenerator1T, typename PwmGenerator2T>
    DRV8251A(PwmGenerator1T&, PwmGenerator2T&, bool, eBrakeMode) -> DRV8251A<PwmGenerator1T, PwmGenerator2T>;

}  // namespace MotorDrivers

#endif  // ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP