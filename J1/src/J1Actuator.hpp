#ifndef J34_HPP
#define J34_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
// TODO rm IFX9201SG
#include "rover_lib2/motor_drivers/IFX007T.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
// TODO rm ATM222X
#include "rover_lib2/sensors/encoder/AMT222A.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/filters/none.hpp"

#include <algorithm>

DEFINE_LOG_NODE(J1Actuator, Logger::eNodeState::OFF);

class J1Actuator
{
    static constexpr float CONTROL_LOOP_FREQUENCY_HZ = 1000.0F;
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = static_cast<uint64_t>(ROUND(1'000'000.0F / CONTROL_LOOP_FREQUENCY_HZ));
    static constexpr float MAX_MOTOR_SPEED_RAD_S = 0.17F;
    static_assert(MAX_MOTOR_SPEED_RAD_S >= 0.0F);
    static constexpr float RAD_TO_M = 0.05026F / (2.0F * std::numbers::pi_v<float>);

    static constexpr float J1_MIN_JOINT_LIMIT = degToRad(-360.0F);
    static constexpr float J1_MAX_JOINT_LIMIT = degToRad(360.0F);
    static_assert(J1_MIN_JOINT_LIMIT <= J1_MAX_JOINT_LIMIT);

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

        __motorDriver.setMaxVoltage(ALIM_VOLTAGE, MAX_MOTOR_VOLTAGE);
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        _j1.update();

        // TODO redo switch
        switch (_currentState)
        {
            default:
                ASSERT_MSG("Shouldn't fall here 0_0");
                [[fallthrough]];
            case eState::RUNNING:
                this->runningUpdateLoop();
                break;
        }
    }

    void runningUpdateLoop()
    {
        float speedCmdJ1 = _j1SpeedGoal;

        if (_j1.getPosition() <= J1_MIN_JOINT_LIMIT)
        {
            speedCmdJ1 = std::clamp(speedCmdJ1, 0.0F, MAX_MOTOR_SPEED_RAD_S);
        }
        else if (_j1.getPosition() >= J1_MAX_JOINT_LIMIT)
        {
            speedCmdJ1 = std::clamp(speedCmdJ1, -MAX_MOTOR_SPEED_RAD_S, 0.0F);
        }

        _j1.setSpeed(speedCmdJ1);
    }

    void setSpeed(float speedJ1_)
    {
        _j1SpeedGoal = speedJ1_;
    }

    float getSpeed() const
    {
        return __j1_encoder.getSpeed() * RAD_TO_M;
    }

    float getPositions(void) const
    {
        return __j1_encoder.getPosition() * RAD_TO_M;
    }

    void calib(float posJ1_)
    {
        __j1_encoder.calib(posJ1_ * std::numbers::pi_v<float>);
    }

  private:
    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _j1SpeedGoal = 0.0F;
    float _j1CurrentPosition = 0.0F;
    float _j1CurrentSpeed = 0.0F;

    eState _currentState = eState::RUNNING;
    OneShotTimer<uint64_t, &Time::millis> _timerWaitAfterCalib = {0};

    Filters::LowPassEMA __filterJ1Speed = {0.1F, 0.0F};
    Filters::LowPassEMA __filterJ1Position = {0.1F, 0.0F};

    PWMGenerators::MCPWMTimer __j1_pwmGeneratorTimer = {1'000UL, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0};
    SPIBus __spi = SPIBus(spi_host_device_t::SPI2_HOST, PIN_ENC_MOSI, PIN_ENC_MISO, PIN_ENC_CLK, 32U);
    // ===========================================================================================================================
    // Motor
    PWMGenerators::MCPWM __pwmBridgeA = {PIN_MOTOR_A_IN, __j1_pwmGeneratorTimer};
    IO::DigitalOutput __bridgeAEn = IO::DigitalOutput(PIN_MOTOR_A_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);
    PWMGenerators::MCPWM __pwmBridgeB = {PIN_MOTOR_B_IN, __j1_pwmGeneratorTimer};
    IO::DigitalOutput __bridgeBEn = IO::DigitalOutput(PIN_MOTOR_B_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);

    MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM> __motorDriver
        = {__bridgeAEn, __pwmBridgeA, __bridgeBEn, __pwmBridgeB, true};

    // Encoder
    Encoders::AMT222A<Filters::None, Filters::None> __j1_encoder = {__spi, PIN_ENC_CS, false};

    // Controller
    Controllers::PID __j1_controllerSpeed = {175.0F, 0.0F, 0.0F, 100.0F, 25'000ULL};

    Actuators::DC<MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM>,
                  Encoders::AMT222A<Filters::None, Filters::None>,
                  Controllers::None,
                  Controllers::PID>
        _j1 = {Actuators::eControlType::SPEED,
               Actuators::eFeedbackType::CLOSE_LOOP,
               __motorDriver,
               &__j1_encoder,
               nullptr,
               &__j1_controllerSpeed};
};

#endif  // J1_HPP
