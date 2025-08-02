#ifndef JLENCODER_HPP
#define JLENCODER_HPP

#include <rover_lib2/filters/low_pass_EMA.hpp>
#include <rover_lib2/sensors/encoder/AMT222X.hpp>
#include <rover_lib2/sensors/encoder/encoder.hpp>

namespace Encoders
{
    class JL
    {
        // From mec team: 50.26mm per turn
        static constexpr float RATIO = 1.0F; // Ratio is 1:1 because joint is linear. RAD_TO_M and M_TO_RAD are used to convert between
        static constexpr float RAD_TO_M = 0.05026F / (2.0F * std::numbers::pi_v<float>);
        static constexpr float M_TO_RAD = 1.0F / RAD_TO_M;

      public:
        void init()
        {
            _encoder.init();
        }

        void update()
        {
            _encoder.update();
        }

        bool dataIsValid() const
        {
            return _encoder.dataIsValid();
        }

        float getPosition() const
        {
            return _encoder.getPosition() * RAD_TO_M;
        }

        float getSpeed() const
        {
            return _encoder.getSpeed() * RAD_TO_M;
        }

        void calib(float offset_)
        {
            _encoder.calib(offset_ * M_TO_RAD);
        }

      private:
        SPIBus __spiBus = SPIBus(spi_host_device_t::SPI2_HOST, PIN_ENC_MOSI, PIN_ENC_MISO, PIN_ENC_CLK, 32U);
        Filters::LowPassEMA __lowPassPos = Filters::LowPassEMA(0.6);
        Filters::LowPassEMA __lowPassSpeed = Filters::LowPassEMA(0.05);
        Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA> _encoder
            = {__spiBus, PIN_ENC_CS, "JL", true, 1.0F, __lowPassPos, __lowPassSpeed};

        VALIDATE_CONCEPT(Encoder, JL);
    };

}  // namespace Encoders
#endif  // JLENCODER_HPP
