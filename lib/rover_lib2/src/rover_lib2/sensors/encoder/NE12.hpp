#ifndef ROVER_LIB2_SENSORS_ENCODER_NE12_HPP
#define ROVER_LIB2_SENSORS_ENCODER_NE12_HPP

#include <numbers>
#include <cstdint>

#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/IO/digital_input.hpp"
#include "rover_lib2/helpers/chrono.hpp"
#include "rover_lib2/filters/none.hpp"

namespace Encoders
{
    template<Filters::Filter FilterPosT = Filters::None, Filters::Filter FilterSpeedT = Filters::None>
    class NE12
    {
        static constexpr uint64_t MIN_TIME_BETWEEN_SPEED_CALC_US = 20'000ULL;  // 20 ms

      public:
        NE12(gpio_num_t channelA_, gpio_num_t channelB_, float countPerRev_):
            _ioA(channelA_),
            _ioB(channelB_),
            _countPerRev(countPerRev_)
        {
        }

        void init()
        {
            uint8_t A = (_ioA.read() == IO::eIOState::HIGH_);
            uint8_t B = (_ioB.read() == IO::eIOState::HIGH_);

            _lastState = (A << 1) | B;

            _dtSpeedCalc.restart();
        }

        void update()
        {
            // ----------- Quadrature decoding -----------
            uint8_t A = (_ioA.read() == IO::eIOState::HIGH_);
            uint8_t B = (_ioB.read() == IO::eIOState::HIGH_);

            uint8_t state = (A << 1) | B;

            _stepCounter += table[_lastState][state];
            _lastState = state;

            // ----------- Raw position -----------
            float currentPosTemp = (static_cast<float>(_stepCounter) / _countPerRev) * 2.0f * std::numbers::pi_v<float>;

            // ----------- Position filtering -----------
            if (_isFirstRead)
            {
                _posFilter.reset(currentPosTemp);
                _currentPosition = currentPosTemp;
                _lastPosition = _currentPosition;
            }
            else
            {
                _currentPosition = _posFilter.addValue(currentPosTemp);
            }

            // ----------- Speed update (timed) -----------
            uint64_t dt_us = _dtSpeedCalc.getTime();

            if (dt_us >= MIN_TIME_BETWEEN_SPEED_CALC_US)
            {
                float dt = static_cast<float>(dt_us) * 1e-6f;

                float delta = _currentPosition - _lastPosition;

                // Wrap-around correction
                if (delta > std::numbers::pi_v<float>)
                    delta -= 2.0f * std::numbers::pi_v<float>;
                else if (delta < -std::numbers::pi_v<float>)
                    delta += 2.0f * std::numbers::pi_v<float>;

                float currentSpeedTemp = delta / dt;

                if (_isFirstRead)
                {
                    _posSpeed.reset(currentSpeedTemp);
                    _speed = currentSpeedTemp;
                }
                else
                {
                    _speed = _posSpeed.addValue(currentSpeedTemp);
                }

                _lastPosition = _currentPosition;
                _dtSpeedCalc.restart();
                _isFirstRead = false;
            }
        }

        bool dataIsValid() const
        {
            return _countPerRev > 0;
        }

        float getPosition() const
        {
            return _currentPosition + _offset;
        }

        float getSpeed() const
        {
            return _speed;
        }

        void calib(float offset_)
        {
            // Define current position as reference
            _offset = offset_ - _currentPosition;
        }

      private:
        IO::DigitalInput _ioA;
        IO::DigitalInput _ioB;

        int64_t _stepCounter = 0;
        uint8_t _lastState = 0;

        float _countPerRev = 1.0f;

        // Position & speed
        float _currentPosition = 0.0f;
        float _lastPosition = 0.0f;
        float _speed = 0.0f;

        float _offset = 0.0f;

        bool _isFirstRead = true;

        Chrono<uint64_t, &Time::micros> _dtSpeedCalc;
        FilterPosT _posFilter;
        FilterSpeedT _posSpeed;

        // Quadrature lookup table
        static constexpr int8_t table[4][4] = {{0, +1, -1, 0}, {-1, 0, 0, +1}, {+1, 0, 0, -1}, {0, -1, +1, 0}};

        VALIDATE_CONCEPT(Encoder, NE12);
    };

}  // namespace Encoders

#endif