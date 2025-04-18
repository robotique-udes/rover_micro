#include <Arduino.h>
#include "rover_lib2/LED/led_blinker.hpp"
#include "rover_lib2/sensors/encoder/AMT222A.hpp"
#include "rover_lib2/sensors/push_button.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_48;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t PIN_SPI_SCLK = GPIO_NUM_47;
constexpr gpio_num_t PIN_SPI_CS = GPIO_NUM_14;

constexpr std::array<uint8_t, 2U> READ_CMD = {0x00, 0x00};
constexpr std::array<uint8_t, 2U> RESET_CMD = {0x00, 0x60};

void setup()
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    LED::LedBlinkerSoft ledUser(IO::DigitalOutput(GPIO_NUM_6), LED::BlinkPatterns::DOUBLE_FLASH, 25U);
    ledUser.init();

    PushButton calib(GPIO_NUM_40);

    SPIBus spi(spi_host_device_t::SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK, 32U);
    AMT222A encoder(spi, PIN_SPI_CS, true);
    encoder.init();

    LoopTimer<uint64_t, Time::millis> printLoop(100);
    for (EVER)
    {
        ledUser.update();
        encoder.update();

        if (printLoop.isReady())
        {
            LOG_INFO(Logger::Nodes::Main,
                     "Data valid: %d, Current position: %f, current speed: %f",
                     encoder.dataIsValid(),
                     encoder.getPosition(),
                     encoder.getSpeed());
        }

        if (calib.isClicked())
        {
            encoder.calib(HALF_PI);
        }
    }
}

void loop() {}
