#ifndef J34_HPP
#define J34_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
// TODO rm ATM222X
#include "rover_lib2/sensors/encoder/AMT222X.hpp"
#include "rover_lib2/sensors/encoder/AMT222A.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

#include <algorithm>

DEFINE_LOG_NODE(J1Actuator, Logger::eNodeState::OFF);

class J1Actuator
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

    static constexpr float ZERO_ERROR_EPSILON = 0.01F;
    static constexpr uint64_t WAIT_TIME_AFTER_CALIB_MS = 500ULL;

    enum class eState : uint8_t
    {
        RUNNING,
        CALIB_REQUESTED,
        WAIT_ON_CALIB,
    };

  public:
    void init()
    {
        _j1.setJointLimit(std::nullopt, std::nullopt);

        _j1.setSpeed(0.0F);

        _j1.setMaxSpeed(MAX_MOTOR_SPEED_RAD_S);

        _j1.init();
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        _j1.update();

        _j1CurrentPosition = _j1.getPosition();

        _j1CurrentSpeed = _j1.getSpeed();

        switch (_currentState)
        {
            default:
                ASSERT_MSG("Shouldn't fall here 0_0");
                [[fallthrough]];
            case eState::RUNNING:
                this->runningUpdateLoop();
                break;

            case eState::CALIB_REQUESTED:
                _j1.setSpeed(0.0F);
                _j1.calib(_j1_requestedCalibPos);

                _timerWaitAfterCalib = OneShotTimer<uint64_t, &Time::millis>{WAIT_TIME_AFTER_CALIB_MS};

                _currentState = eState::WAIT_ON_CALIB;
                break;

            case eState::WAIT_ON_CALIB:
                if (_timerWaitAfterCalib.isReady())
                {
                    _currentState = eState::RUNNING;
                }
                break;
        }
    }

    void runningUpdateLoop()
    {
        float speedCmdJ1 = _j1SpeedGoal;

        if (_j1CurrentPosition <= J3_MIN_JOINT_LIMIT)
        {
            speedCmdJ1 = std::clamp(speedCmdJ1, 0.0F, MAX_MOTOR_SPEED_RAD_S);
        }
        else if (_j1CurrentPosition >= J3_MAX_JOINT_LIMIT)
        {
            speedCmdJ1 = std::clamp(speedCmdJ1, -MAX_MOTOR_SPEED_RAD_S, 0.0F);
        }

        if (speedCmdJ1 > MAX_MOTOR_SPEED_RAD_S)
        {
            float scaleFactor = MAX_MOTOR_SPEED_RAD_S / speedCmdJ1;
            speedCmdJ1 *= scaleFactor;
        }

        _j1.setSpeed(speedCmdJ1);
    }

    void setSpeeds(float speedJ1_)
    {
        _j1SpeedGoal = speedJ1_;
    }

    void getSpeeds(float& speedJ1_) const
    {
        speedJ1_ = _j1CurrentSpeed;
    }

    void getPositions(float& posJ1_) const
    {
        posJ1_ = _j1CurrentPosition;
    }

    void calib(float posJ1_)
    {
        _j1_requestedCalibPos = posJ1_;
        _currentState = eState::CALIB_REQUESTED;
    }

  private:
    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _j1SpeedGoal = 0.0F;
    float _j1CurrentPosition = 0.0F;
    float _j1CurrentSpeed = 0.0F;

    float _j1_requestedCalibPos = 0.0F;

    eState _currentState = eState::RUNNING;
    OneShotTimer<uint64_t, &Time::millis> _timerWaitAfterCalib = {0};

    // ===========================================================================================================================
    // Generic Objects
    // ===========================================================================================================================
    PWMGenerators::MCPWMTimer __j1_pwmGeneratorTimer = {1'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0};
    SPIBus __spi = SPIBus(spi_host_device_t::SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, 32U);
    // ===========================================================================================================================
    // Motor
    PWMGenerators::MCPWM __j1_pwmGen = {PIN_J34_L_PWM, __j1_pwmGeneratorTimer};
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> __j1_driver = {__j1_pwmGen, PIN_J34_L_DIR, false};

    // Encoder
    Encoders::AMT222A __j1_encoder = {__spi, PIN_J34_L_CS, false};

    // Controller
    // TODO put these values in constexpr
    Controllers::PID __j1_controllerSpeed = {50.0F, 12.5F, 0.1F, 100.0F, 20'000ULL};

    Actuators::DC<MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>, Encoders::AMT222A, Controllers::None, Controllers::PID> _j1
        = {Actuators::eControlType::SPEED,
           Actuators::eFeedbackType::CLOSE_LOOP,
           __j1_driver,
           &__j1_encoder,
           nullptr,
           &__j1_controllerSpeed};
};

#endif  // J34_HPP
