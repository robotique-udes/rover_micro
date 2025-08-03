#ifndef J2_DEVICE_HPP
#define J2_DEVICE_HPP

#include "J2Actuator.hpp"

#include <rover_lib2/sensors/push_button.hpp>

DEFINE_LOG_NODE(J2Device, Logger::eNodeState::ON);

class J2Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 1'000ULL;

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.75F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.0F;

    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m

  public:
    J2Device(Stream* motorSerial_ = nullptr):
        _motorSerial(motorSerial_),
        _j2(motorSerial_)
    {
    }

    void init()
    {
        _j2.init();
        _j2.setSpeed(0.0F);
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        _j2.update();

        if (_pbCalib.isClicked())
        {
            _j2.setSpeed(FULL_STOP_SPEED);

            constexpr uint64_t CALIB_STOP_TIME = 1000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _j2.update();

                if (!IN_ERROR(_j2.getSpeed(), FULL_STOP_SPEED_ERROR_TELORANCE, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _j2.calib(CALIB_POSITION);
        }

        if (_pbJogPlus.isClicked())
        {
            _j2.setSpeed(PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_pbJogNeg.isClicked())
        {
            _j2.setSpeed(-PUSH_BUTTON_SPEED_RAD_S);
        }
        else
        {
            _j2.setSpeed(FULL_STOP_SPEED);
        }
    }

  private:
    PushButton _pbJogPlus = {PIN_PB_PLUS};
    PushButton _pbJogNeg = {PIN_PB_NEG};
    PushButton _pbCalib = {PIN_PB_CALIB};

    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {LOOP_PERIOD_US};

    Stream* _motorSerial = nullptr;

    J2Actuator _j2;
    float _j2SpeedGoal = 0.0F;

    VALIDATE_CONCEPT(RoverObject, J2Device);
};

#endif