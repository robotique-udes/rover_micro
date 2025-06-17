#ifndef PID_HPP
#define PID_HPP

#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <complex>
#include <cmath>

namespace Controllers
{
    static constexpr uint64_t PERIOD_SPEED_CALC_US = 1'000ULL;

    // concept Controller
    class PID
    {
      public:
        PID(float kp_, float ki_, float kd_, float integralLimit_, uint64_t dtCalcPeriod_ = PERIOD_SPEED_CALC_US):
            _speedCalcPeriod(dtCalcPeriod_),
            _kp(kp_),
            _ki(ki_),
            _kd(kd_),
            _integralLimit(std::abs(integralLimit_))
        {
            _lastMeasureTime = Time::micros();
            this->reset();
        }

        float computeCommand(float input_, float target_)
        {
            float error = target_ - input_;

            if (std::isnan(error))
            {
                error = 0.0f;
            }
            if (std::isinf(error))
            {
                error = std::numeric_limits<float>::max();
            }

            _cmdI += _ki * error;
            _cmdI = std::clamp(_cmdI, -_integralLimit, _integralLimit);

            float cmdP = _kp * error;

            float cmdD = 0.0f;
            uint64_t currentTime = Time::micros();
            if (currentTime - _lastMeasureTime < _speedCalcPeriod)
            {
                cmdD = _lastcmdD;
            }
            else
            {
                uint64_t dt = currentTime - _lastMeasureTime;
                _lastMeasureTime = currentTime;

                float dtSec = static_cast<float>(dt) / 1'000'000.0f;
                if (dtSec != 0.0F)
                {
                    cmdD = _kd * (error - _previousError) / dtSec;
                    _lastcmdD = cmdD;
                }
                else
                {
                    cmdD = _lastcmdD;
                }
            }

            if (std::isnan(cmdP))
            {
                cmdP = 0.0f;
            }
            if (std::isnan(_cmdI))
            {
                _cmdI = 0.0f;
            }
            if (std::isnan(cmdD))
            {
                cmdD = 0.0f;
            }

            _previousError = error;

            return cmdP + _cmdI + cmdD;
        }

        void reset()
        {
            _cmdI = 0.0f;
            _previousError = 0.0f;
        }

        void setGains(float kp_, float ki_, float kd_)
        {
            _kp = kp_;
            _ki = ki_;
            _kd = kd_;
        }

      private:
        const uint64_t _speedCalcPeriod;

        float _kp = 0.0f;
        float _ki = 0.0f;
        float _kd = 0.0f;
        float _integralLimit = 0.0f;

        float _cmdI = 0.0f;
        float _lastcmdD = 0.0f;
        float _previousError = 0.0f;
        uint64_t _lastMeasureTime = 0ULL;
    };

}  // namespace Controllers

#endif  // PID_HPP
