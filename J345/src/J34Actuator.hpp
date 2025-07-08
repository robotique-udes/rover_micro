#ifndef J34_HPP
#define J34_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/encoder/AMT222X.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

#include <algorithm>

DEFINE_LOG_NODE(J34Actuator, Logger::eNodeState::ON);

class J34Actuator
{
    static constexpr float CONTROL_LOOP_FREQUENCY_HZ = 1000.0F;
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = static_cast<uint64_t>(ROUND(1'000'000.0F / CONTROL_LOOP_FREQUENCY_HZ));
    static constexpr float MAX_MOTOR_SPEED_RAD_S = 0.7F;
    static_assert(MAX_MOTOR_SPEED_RAD_S >= 0.0F);

    static constexpr float J3_MIN_JOINT_LIMIT = degToRad(-40.0F);
    static constexpr float J3_MAX_JOINT_LIMIT = degToRad(40.0F);
    static_assert(J3_MIN_JOINT_LIMIT <= J3_MAX_JOINT_LIMIT);

    static constexpr float J4_MIN_JOINT_LIMIT = degToRad(-5.0F * 360.0F);
    static constexpr float J4_MAX_JOINT_LIMIT = degToRad(5.0F * 360.0F);
    static_assert(J4_MIN_JOINT_LIMIT <= J4_MAX_JOINT_LIMIT);

    enum class eState : uint8_t
    {
        RUNNING,
        CALIB_REQUESTED,
        WAIT_ON_STOP,
        WAIT_ON_CALIB,
    };

  public:
    J34Actuator() = default;
    void init()
    {
        _j34L.setJointLimit(std::nullopt, std::nullopt);  // Joint limit needs to be managed by differential logic instead
        _j34R.setJointLimit(std::nullopt, std::nullopt);  // Joint limit needs to be managed by differential logic instead

        _j34L.setSpeed(0.0F);
        _j34R.setSpeed(0.0F);

        _j34L.setMaxSpeed(MAX_MOTOR_SPEED_RAD_S);
        _j34R.setMaxSpeed(MAX_MOTOR_SPEED_RAD_S);

        _j34L.init();
        _j34R.init();
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        switch (_currentState)
        {
            case eState::CALIB_REQUESTED:
                this->setSpeeds(0.0F, 0.0F);
                _currentState = eState::WAIT_ON_STOP;
                break;
            case eState::WAIT_ON_STOP:
            {
                float speedJ3 = 1.0F;
                float speedJ4 = 1.0F;
                this->getSpeeds(speedJ3, speedJ4);

                if (IN_ERROR(speedJ3, 0.01F, 0.0F) && IN_ERROR(speedJ4, 0.01F, 0.0F))
                {
                    _currentState = eState::WAIT_ON_CALIB;
                }
            }
            break;

            case eState::WAIT_ON_CALIB:
                _j34L.calib(_j34L_requestedCalibPos);
                _j34R.calib(_j34R_requestedCalibPos);
                _currentState = eState::RUNNING;
                delay(500);
                break;

            case eState::RUNNING:
                [[fallthrough]];
            default:
                break;
        }

        _j34L.update();
        _j34R.update();

        _j3CurrentPosition = (_j34L.getPosition() + _j34R.getPosition()) / 2.0F;
        _j4CurrentPosition = (_j34L.getPosition() - _j34R.getPosition()) / 2.0F;

        _j3CurrentSpeed = (_j34L.getSpeed() + _j34R.getSpeed()) / 2.0F;
        _j4CurrentSpeed = (_j34L.getSpeed() - _j34R.getSpeed()) / 2.0F;

        float speedCmdJ3 = _j3SpeedGoal;
        float speedCmdJ4 = _j4SpeedGoal;

        if (_j3CurrentPosition <= J3_MIN_JOINT_LIMIT)
        {
            speedCmdJ3 = std::clamp(speedCmdJ3, 0.0F, MAX_MOTOR_SPEED_RAD_S);
        }
        else if (_j3CurrentPosition >= J3_MAX_JOINT_LIMIT)
        {
            speedCmdJ3 = std::clamp(speedCmdJ3, -MAX_MOTOR_SPEED_RAD_S, 0.0F);
        }

        if (_j4CurrentPosition <= J4_MIN_JOINT_LIMIT)
        {
            speedCmdJ4 = std::clamp(speedCmdJ4, 0.0F, 2.0F * MAX_MOTOR_SPEED_RAD_S);
        }
        else if (_j4CurrentPosition >= J4_MAX_JOINT_LIMIT)
        {
            speedCmdJ4 = std::clamp(speedCmdJ4, -2.0F * MAX_MOTOR_SPEED_RAD_S, 0.0F);
        }

        float j34LSpeed = speedCmdJ3 + speedCmdJ4 / 2.0F;
        float j34RSpeed = speedCmdJ3 - speedCmdJ4 / 2.0F;

        float maxSpeed = std::max(std::abs(j34LSpeed), std::abs(j34RSpeed));
        if (maxSpeed > MAX_MOTOR_SPEED_RAD_S)
        {
            float scaleFactor = MAX_MOTOR_SPEED_RAD_S / maxSpeed;
            j34LSpeed *= scaleFactor;
            j34RSpeed *= scaleFactor;
        }

        _j34L.setSpeed(j34LSpeed);
        _j34R.setSpeed(j34RSpeed);
    }

    void setSpeeds(float speedJ3_, float speedJ4_)
    {
        _j3SpeedGoal = speedJ3_;
        _j4SpeedGoal = speedJ4_;
    }

    void getSpeeds(float& speedJ3_, float& speedJ4_) const
    {
        speedJ3_ = _j3CurrentSpeed;
        speedJ4_ = _j4CurrentSpeed;
    }

    void getPositions(float& posJ3_, float& posJ4_) const
    {
        posJ3_ = _j3CurrentPosition;
        posJ4_ = _j4CurrentPosition;
    }

    void calib(float posJ3_, float posJ4_)
    {
        _currentState = eState::CALIB_REQUESTED;

        _j34R_requestedCalibPos = posJ3_;
        _j34L_requestedCalibPos = posJ4_;
        // _j34L.calib(posJ3_);
        // _j34R.calib(posJ4_);
    }

  private:
    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _j3SpeedGoal = 0.0F;
    float _j4SpeedGoal = 0.0F;
    float _j3CurrentPosition = 0.0F;
    float _j4CurrentPosition = 0.0F;
    float _j3CurrentSpeed = 0.0F;
    float _j4CurrentSpeed = 0.0F;

    float _j34R_requestedCalibPos = 0.0F;
    float _j34L_requestedCalibPos = 0.0F;

    eState _currentState = eState::RUNNING;

    // ===========================================================================================================================
    // Generic Objects
    // ===========================================================================================================================
    PWMGenerators::MCPWMTimer __j34_pwmGeneratorTimer = {1'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0};
    SPIBus _spi = SPIBus(spi_host_device_t::SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, 32U);

    // ===========================================================================================================================
    // J34_L Config
    // ===========================================================================================================================
    // Motor
    PWMGenerators::MCPWM __j34L_pwmGen = {PIN_J34_L_PWM, __j34_pwmGeneratorTimer};
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> __j34L_driver = {__j34L_pwmGen, PIN_J34_L_DIR, false};

    // Encoder
    Filters::LowPassEMA __j34L_encFilterPos = Filters::LowPassEMA(0.05F);
    Filters::LowPassEMA __j34L_encFilterSpeed = Filters::LowPassEMA(0.4F);
    Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA> __j34L_encoder
        = {_spi, PIN_J34_L_CS, "J34L", __j34L_encFilterPos, __j34L_encFilterSpeed, true};

    // Controller
    Controllers::PID __j34L_controllerSpeed = {50.0F, 12.5F, 0.1F, 100.0F, 20'000ULL};

    Actuators::DC<MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>,
                  Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA>,
                  Controllers::None,
                  Controllers::PID>
        _j34L = {Actuators::eControlType::SPEED,
                 Actuators::eFeedbackType::CLOSE_LOOP,
                 __j34L_driver,
                 &__j34L_encoder,
                 nullptr,
                 &__j34L_controllerSpeed};

    // ===========================================================================================================================
    // J34_R Config
    // ===========================================================================================================================
    // Motor
    PWMGenerators::MCPWM __j34R_pwmGen = {PIN_J34_R_PWM, __j34_pwmGeneratorTimer};
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> __j34R_driver = {__j34R_pwmGen, PIN_J34_R_DIR, true};

    // Encoder
    Filters::LowPassEMA __j34R_encFilterPos = Filters::LowPassEMA(0.05F);
    Filters::LowPassEMA __j34R_encFilterSpeed = Filters::LowPassEMA(0.4F);
    Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA> __j34R_encoder
        = {_spi, PIN_J34_R_CS, "J34R", __j34R_encFilterPos, __j34R_encFilterSpeed, false};

    // Controller
    Controllers::PID __j34R_controllerSpeed = {50.0F, 12.5F, 0.1F, 100.0F, 20'000ULL};

    Actuators::DC<MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>,
                  Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA>,
                  Controllers::None,
                  Controllers::PID>
        _j34R = {Actuators::eControlType::SPEED,
                 Actuators::eFeedbackType::CLOSE_LOOP,
                 __j34R_driver,
                 &__j34R_encoder,
                 nullptr,
                 &__j34R_controllerSpeed};
};

#endif  // J34_HPP
