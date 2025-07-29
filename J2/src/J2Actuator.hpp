#ifndef J2_ACTUATOR
#define J2_ACTUATOR

#include "config.hpp"
#include "rover_lib2/actuators/AK10-9/AK10-9.hpp"
#include "rover_lib2/sensors/encoder/AMT222X.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/filters/none.hpp"

#include <algorithm>

DEFINE_LOG_NODE(J1Actuator, Logger::eNodeState::ON);
DEFINE_LOG_NODE(J1ActuatorPlot, Logger::eNodeState::OFF);

class J2Actuator
{
    static constexpr float CONTROL_LOOP_FREQUENCY_HZ = 1000.0F;
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = static_cast<uint64_t>(ROUND(1'000'000.0F / CONTROL_LOOP_FREQUENCY_HZ));
    static constexpr float MAX_MOTOR_SPEED_RAD_S = 0.8F;
    static_assert(MAX_MOTOR_SPEED_RAD_S >= 0.0F);
    static constexpr float RAD_TO_M = 0.05026F / (2.0F * std::numbers::pi_v<float>);

    static constexpr float J2_MIN_JOINT_LIMIT = degToRad(-360.0F);
    static constexpr float J2_MAX_JOINT_LIMIT = degToRad(360.0F);
    static_assert(J2_MIN_JOINT_LIMIT <= J2_MAX_JOINT_LIMIT);

    static constexpr float ZERO_ERROR_EPSILON = 0.01F;
    static constexpr uint64_t WAIT_TIME_AFTER_CALIB_MS = 500ULL;

  public:
    void init()
    {
        _j2.setJointLimit(std::nullopt, std::nullopt);

        _j2.setSpeed(0.0F);

        _j2.setMaxSpeed(MAX_MOTOR_SPEED_RAD_S);

        _j2.init();

        _motorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        _j2.update();

        if (_j2.getPosition() <= J2_MIN_JOINT_LIMIT)
        {
            _j2.setSpeed(std::clamp(_j2SpeedGoal, 0.0F, MAX_MOTOR_SPEED_RAD_S));
        }
        else if (_j2.getPosition() >= J2_MAX_JOINT_LIMIT)
        {
            _j2.setSpeed(std::clamp(_j2SpeedGoal, -MAX_MOTOR_SPEED_RAD_S, 0.0F));
        }
        else
        {
            _j2.setSpeed(_j2SpeedGoal);
        }
    }

    void setSpeed(float goalSpeed_)
    {
        _j2SpeedGoal = goalSpeed_;
    }

    float getSpeed() const
    {
        return _j2Encoder.getSpeed() * RAD_TO_M;
    }

    float getPosition() const
    {
        return _j2Encoder.getPosition() * RAD_TO_M;
    }

    void calib(float offset_)
    {
        _j2Encoder.calib(offset_ * std::numbers::pi_v<float>);
    }

  private:
    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _j2SpeedGoal = 0.0F;
    float _j2CurrentPosition = 0.0F;
    float _j2CurrentSpeed = 0.0F;

    HardwareSerial _motorSerial = HardwareSerial(UART_PORT);

    OneShotTimer<uint64_t, &Time::millis> _timerWaitAfterCalib = {0};

    Filters::LowPassEMA _j2SpeedFilter = {0.1F, 0.0F};
    Filters::LowPassEMA _j2PositionFilter = {0.1F, 0.0F};

    SPIBus __spi = SPIBus(spi_host_device_t::SPI2_HOST, PIN_ENC_MOSI, PIN_ENC_MISO, PIN_ENC_CLK, 32U);

    Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA> _j2Encoder
        = {__spi, PIN_ENC_CS, "J2", _j2PositionFilter, _j2SpeedFilter, false};

    Controllers::PID __j1_controllerSpeed = {75.0F, 150.0F, 20.0F, 100.0F, 25'000ULL};

    Actuators::AK109<Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA>, Controllers::None, Controllers::PID> _j2
        = {Actuators::eControlType::SPEED, &_motorSerial, &_j2Encoder, nullptr, &__j1_controllerSpeed};
};

#endif