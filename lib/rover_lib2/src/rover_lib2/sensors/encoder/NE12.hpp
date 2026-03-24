#ifndef ROVER_LIB2_SENSORS_ENCODER_NE12_HPP
#define ROVER_LIB2_SENSORS_ENCODER_NE12_HPP

#include <numbers>

#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/IO/digital_input.hpp"

namespace Encoders
{
    class NE12
    {
      public:
        NE12(gpio_num_t channelA_, gpio_num_t channelB_, float countPerRev_):
            _ioA(channelA_),
            _ioB(channelB_),
            _countPerRev(countPerRev_)
        {
        }

        void init()
        {
            ASSERT_MSG("Interface");
        }

        void update()
        {
            uint8_t A = (_ioA.read() == IO::eIOState::HIGH_);
            uint8_t B = (_ioB.read() == IO::eIOState::HIGH_);

            uint8_t state = (A << 1) | B;

            this->_stepCounter += table[this->_lastState][state];
            this->_lastState = state;
        }

        bool dataIsValid() const
        {
            ASSERT_MSG("Interface");
            return false;
        }

        float getPosition() const
        {
            return _stepCounter / this->_countPerRev * 2 * std::numbers::pi_v<float>;
        }

        float getSpeed() const
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void calib(float /*offset_*/)
        {
            ASSERT_MSG("Interface");
        }

      private:
        IO::DigitalInput _ioA;
        IO::DigitalInput _ioB;

        int64_t _stepCounter = 0;
        uint8_t _lastState = 0U;
        uint16_t _countPerRev = 0;

        // Direction is B --> A (positive), A --> B (negative)
        static constexpr int8_t table[4][4] = {
            {0, +1, -1, 0},  // 00
            {-1, 0, 0, +1},  // 01
            {+1, 0, 0, -1},  // 10
            {0, -1, +1, 0}   // 11
        };

        VALIDATE_CONCEPT(Encoder, NE12);
    };
}  // namespace Encoders

#endif  // ROVER_LIB2_SENSORS_ENCODER_NE12_HPP