#ifndef IFX9201SG_HPP
#define IFX9201SG_HPP

#include "motor_driver.hpp"

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/IO/digital_output.hpp"

#include <optional>

DEFINE_LOG_NODE(IFX9201SG, Logger::eNodeState::ON);

template<typename PWMGenerator_T>
class IFX9201SG : public MotorDriver<IFX9201SG<PWMGenerator_T>>
{
    VALIDATE_BASE_TYPE(PWMGenerators::PWMGeneratorT, PWMGenerator_T);

    static constexpr float ZERO_ERROR_TOLERANCE = 0.1F;  // When command are considered null

  public:
    IFX9201SG(PWMGenerator_T& pwmGenerator_,
              gpio_num_t pinDir_,
              bool reversed_,
              gpio_num_t pinDis_ = GPIO_NUM_NC,
              MotorDriverT::eBrakeMode brakeMode_ = MotorDriverT::eBrakeMode::BRAKE):
        _pwmGenerator(pwmGenerator_),
        _ioDir(pinDir_),
        _ioNotEn(pinDis_),
        _enabled(false),
        _reversed(false),
        _goalCmd(0),
        _coastPossible(false),
        _brakeMode(brakeMode_)
    {
        this->setEnabled(false);
        this->setReversed(reversed_);
        _ioDir.write(IO::eIOState::LOW_);

        if (pinDis_ != GPIO_NUM_NC)
        {
            _coastPossible = false;
            this->setBrakeMode(MotorDriverT::eBrakeMode::BRAKE);
        }
        else
        {
            _coastPossible = true;
        }
    }

    void __init(void)
    {
        _pwmGenerator.init();
    }

    void __update(void)
    {
        float duty = std::abs(_goalCmd);
        bool negativeCmd = (_goalCmd < 0.0F);

        LOG_DEBUG(Logger::Nodes::IFX9201SG, "duty: %f, negativeCmd: %s", duty, negativeCmd ? "TRUE" : "FALSE");
        switch (_brakeMode)
        {
            case MotorDriverT::eBrakeMode::BRAKE:
                if (negativeCmd)
                {
                    _ioDir.write(IO::eIOState::HIGH_);
                }
                else
                {
                    _ioDir.write(IO::eIOState::LOW_);
                }

                break;
            case MotorDriverT::eBrakeMode::COAST:
                if (IN_ERROR(duty, ZERO_ERROR_TOLERANCE, 0.0F))
                {
                    _ioNotEn.write(IO::eIOState::HIGH_);
                }
                else if (negativeCmd)
                {
                    _ioDir.write(IO::eIOState::HIGH_);
                }
                else
                {
                    _ioDir.write(IO::eIOState::LOW_);
                }
                break;
        }

        _pwmGenerator.setDutyCycle(duty);
        _pwmGenerator.update();
    }

    void _setCmd(float cmd_)
    {
        _goalCmd = CONSTRAIN(cmd_, -100.0F, 100.0F);
        if (_reversed)
        {
            _goalCmd = -_goalCmd;
        }
    }

    float _getCmd(void)
    {
        if (_reversed)
        {
            return -_goalCmd;
        }
        else
        {
            return _goalCmd;
        }
    }

    void _setEnabled(bool on_)
    {
        if (_enabled == on_)
        {
            return;
        }

        _enabled = on_;
        if (on_)
        {
            _ioNotEn.write(IO::eIOState::LOW_);
        }
        else
        {
            _ioNotEn.write(IO::eIOState::HIGH_);
        }
    }

    bool _isEnabled(void)
    {
        return _enabled;
    }

    void setReversed(bool reversed_)
    {
        _reversed = reversed_;
        this->setCmd(this->getCmd());  // will apply reversed on the internal command value
    }

    bool _isReversed(void)
    {
        return _reversed;
    }

    void _setBrakeMode(MotorDriverT::eBrakeMode mode_)
    {
        if (_brakeMode == mode_)
        {
            return;
        }

        if (!_coastPossible)
        {
            _brakeMode = MotorDriverT::eBrakeMode::BRAKE;
        }
        else
        {
            _brakeMode = mode_;
        }
    }

    MotorDriverT::eBrakeMode _getBrakeMode(void)
    {
        return _brakeMode;
    }

  private:
    PWMGenerator_T& _pwmGenerator;

    IO::DigitalOutput _ioDir;
    IO::DigitalOutput _ioNotEn;

    bool _enabled;
    bool _reversed;
    float _goalCmd;

    bool _coastPossible;
    MotorDriverT::eBrakeMode _brakeMode;
};

template<typename PwmGenerator_T>
IFX9201SG(PwmGenerator_T&, gpio_num_t, bool, gpio_num_t, MotorDriverT::eBrakeMode) -> IFX9201SG<PwmGenerator_T>;

#endif  // IFX9201SG_HPP
