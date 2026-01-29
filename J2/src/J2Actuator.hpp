#ifndef J2_ACTUATOR
#define J2_ACTUATOR

#include "config.hpp"
#include "rover_lib2/actuators/AK60_6/AK60_6.hpp"
#include "rover_lib2/sensors/encoder/AMT222X.hpp"
#include "rover_lib2/filters/low_pass_EMA.hpp"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/filters/none.hpp"

#include <algorithm>

DEFINE_LOG_NODE(J2Actuator, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(J2ActuatorPlot, Logger::eNodeState::OFF);

class J2Actuator
{
    static constexpr float CONTROL_LOOP_FREQUENCY_HZ = 250.0F;
    static constexpr uint64_t CONTROL_LOOP_PERIOD_US = static_cast<uint64_t>(ROUND(1'000'000.0F / CONTROL_LOOP_FREQUENCY_HZ));
    static constexpr float MAX_MOTOR_SPEED_RAD_S = 0.2F;
    static_assert(MAX_MOTOR_SPEED_RAD_S >= 0.0F);

    static constexpr float J2_MIN_JOINT_LIMIT = -2.45F;
    static constexpr float J2_MAX_JOINT_LIMIT = 2.45F;
    static_assert(J2_MIN_JOINT_LIMIT <= J2_MAX_JOINT_LIMIT);

    static constexpr float RATIO_OUTPUT_TO_MOTOR = 100.0F;
    static constexpr float RATIO_OUTPUT_TO_ENCODER = 1.0F / 3.0F;

  public:
    explicit J2Actuator(std::reference_wrapper<Stream> motorSerial_):
        _j2(Actuators::AK60_6::eControlType::SPEED, motorSerial_)
    {
    }

    void init()
    {
        _encoder.init();
        _j2.setSpeed(0.0F);
        _j2.init();
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        _encoder.update();

        float speedCmd = 0.0F;
        if (this->getPosition() <= J2_MIN_JOINT_LIMIT)
        {
            speedCmd = std::clamp(_j2SpeedGoal, 0.0F, MAX_MOTOR_SPEED_RAD_S);
        }
        else if (this->getPosition() >= J2_MAX_JOINT_LIMIT)
        {
            speedCmd = std::clamp(_j2SpeedGoal, -MAX_MOTOR_SPEED_RAD_S, 0.0F);
        }
        else
        {
            speedCmd = _j2SpeedGoal;
        }

        _j2.setSpeed(-10.0F * speedCmd);
        _j2.update();

        LOG_INFO(Logger::Nodes::J2Actuator,
                 "this->getPosition(): %f, this->getSpeed(): %f",
                 this->getPosition(),
                 this->getSpeed());

        LOG_PLOT(Logger::Nodes::J2ActuatorPlot, this->getPosition(), this->getSpeed(), speedCmd);
    }

    void setSpeed(float goalSpeed_)
    {
        _j2SpeedGoal = goalSpeed_;
    }

    float getSpeed() const
    {
        return _encoder.getSpeed();
    }

    float getPosition() const
    {
        return _encoder.getPosition();
    }

    void calib(float offset_)
    {
        _encoder.calib(offset_);
    }

  private:
    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {CONTROL_LOOP_PERIOD_US};

    float _j2SpeedGoal = 0.0F;
    OneShotTimer<uint64_t, &Time::millis> _timerWaitAfterCalib = {0};

    Filters::LowPassEMA _j2SpeedFilter = {0.15F, 0.0F};
    Filters::LowPassEMA _j2PositionFilter = {0.9F, 0.0F};

    SPIBus __spi = SPIBus(spi_host_device_t::SPI2_HOST, PIN_ENC_MOSI, PIN_ENC_MISO, PIN_ENC_CLK, 32U);

    Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA> _encoder
        = {__spi, PIN_ENC_CS, "J2", true, RATIO_OUTPUT_TO_ENCODER, _j2PositionFilter, _j2SpeedFilter};

    Actuators::AK60_6 _j2;
};

#endif
