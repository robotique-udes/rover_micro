/**
 * SPI Test, using J345 J34_L driver as slave device
 */

#include <Arduino.h>
#include "rover_lib2/communication/SPI/SPI_device.hpp"

// Pin definitions for SPI2 on ESP32-S3
constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_48;
constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_21;
constexpr gpio_num_t SPI_SCLK_PIN = GPIO_NUM_47;
constexpr gpio_num_t SPI_CS_PIN = GPIO_NUM_15;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup()
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    SPIBus spi(spi_host_device_t::SPI2_HOST, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN, 32U);
    SPIDevice<2U> motorDriverJ34_L(spi, SPI_CS_PIN, 1'000'000UL, 2U, 2U, SPIDeviceT::eSPIMode::MODE_1);

    delay(1000);
    std::array<uint8_t, 1> rcvData = {};
    std::array<uint8_t, 1> data = {0b1100'1111};
    for (EVER)
    {
        motorDriverJ34_L.writeData(data);
        while (motorDriverJ34_L.readData(rcvData) == SPIDeviceT::eReturnCode::TRANSMISSION_IN_PROGRESS)
        {
        }
        LOG_INFO(Logger::Nodes::Main, "Received 0x%x", rcvData[0]);
    }
}

void loop() {}
