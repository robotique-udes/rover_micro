#ifndef JL_DEVICE_HPP
#define JL_DEVICE_HPP

#include "config.hpp"
#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/IFX007T.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "JLEncoder.hpp"

DEFINE_LOG_NODE(JLDevice, Logger::eNodeState::ON);
class JLDevice
{
    static constexpr uint32_t PWM_FREQUENCY = 1'000UL;
    static constexpr uint32_t CONTROL_LOOP_PERIOD_US = 1'000UL;
    static constexpr float JOG_SPEED = 0.04F;
    static constexpr float FULL_STOP_SPEED = 0.0F;

  public:
    void init()
    {
        _actuator.init();
        _actuator.setSpeed(FULL_STOP_SPEED);
        __motorDriver.setMaxVoltage(ALIM_VOLTAGE, MAX_MOTOR_VOLTAGE);
    }

    void update()
    {
        if (timerControlLoop.isReady())
        {
            _actuator.update();
        }

        if (_pbCalib.isClicked())
        {
            _actuator.setSpeed(0.0);

            constexpr uint64_t CALIB_STOP_TIME = 1000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _actuator.update();

                if (!IN_ERROR(_actuator.getSpeed(), 0.1F, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _actuator.calib(0.25F);
        }

        if (_pbFwd.isClicked())
        {
            _actuator.setSpeed(JOG_SPEED);
        }
        else if (_pbRev.isClicked())
        {
            _actuator.setSpeed(-JOG_SPEED);
        }
        else
        {
            _actuator.setSpeed(FULL_STOP_SPEED);
        }
    }

  private:
    PushButton _pbFwd = PushButton(PIN_PB_FWD);
    PushButton _pbRev = PushButton(PIN_PB_REV);
    PushButton _pbCalib = PushButton(PIN_PB_CALIB);

    LoopTimer<uint64_t, &Time::micros> timerControlLoop{CONTROL_LOOP_PERIOD_US};

    // ===========================================================================================================================
    // Actuator Config
    // ===========================================================================================================================
    PWMGenerators::MCPWMTimer __pwmGen = {PWM_FREQUENCY, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0};
    PWMGenerators::MCPWM __pwmBridgeA = {PIN_MOTOR_A_IN, __pwmGen};
    IO::DigitalOutput __bridgeAEn = IO::DigitalOutput(PIN_MOTOR_A_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);
    PWMGenerators::MCPWM __pwmBridgeB = {PIN_MOTOR_B_IN, __pwmGen};
    IO::DigitalOutput __bridgeBEn = IO::DigitalOutput(PIN_MOTOR_B_EN,
                                                      IO::eIOState::LOW_,
                                                      gpio_mode_t::GPIO_MODE_INPUT_OUTPUT,
                                                      gpio_pull_mode_t::GPIO_PULLDOWN_ONLY);
    MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM> __motorDriver
        = {__bridgeAEn, __pwmBridgeA, __bridgeBEn, __pwmBridgeB, false};

    Encoders::JL __encoder;

    Controllers::PID __pidSpeed = Controllers::PID(1'500.0F, 150.0F, 15.0F, 40.0F, 25'000ULL);

    Actuators::
        DC<MotorDrivers::IFX007T<PWMGenerators::MCPWM, PWMGenerators::MCPWM>, Encoders::JL, Controllers::None, Controllers::PID>
            _actuator
        = {Actuators::eControlType::SPEED, Actuators::eFeedbackType::CLOSE_LOOP, __motorDriver, &__encoder, nullptr, &__pidSpeed};
};

#endif  // JL_DEVICE_HPP
