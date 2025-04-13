/**
 * @file main.hpp
 * @brief Main for quick testing or library implementation.
 *
 * The following folder structure in vscode is recommended to make intellisense work in the lib folder
 * └── lib_developpement
 *   ├── .clang-format
 *   ├── .gitignore
 *   ├── include
 *   ├── .pio
 *   ├── platformio.ini
 *   ├── src
 *   ├── test
 *   └── .vscode
 * └── lib
 *   ├── .clang-format
 *   ├── lib_rover
 *   ├── rover_can2
 *   ├── rover_can_lib
 *   ├── rover_lib2
 *   └── .vscode
 *
 *  [WARNING] For intellisense to work, replace the default .vscode inside the lib folder with the one generated under
 * lib_developpement
 *
 * Backup all your manuel tests under:
 *  lib_developpement/test/manual_embedded
 */

#include <Arduino.h>
#include "rover_lib2/LED/led_blinker.hpp"

#include "rover_can2/rover_can2.hpp"

#include "driver/spi_master.h"

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_48;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t PIN_SPI_SCLK = GPIO_NUM_47;
constexpr gpio_num_t PIN_SPI_NCS = GPIO_NUM_16;

constexpr uint8_t SPI_CMD_RD_REV = 0b1000'0000;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    while (1)
        ;

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);

    gpio_reset_pin(PIN_SPI_SCLK);
    gpio_reset_pin(PIN_SPI_MOSI);
    gpio_reset_pin(PIN_SPI_MISO);

    spi_bus_config_t SpiBusConfig = {.mosi_io_num = PIN_SPI_MOSI,
                                     .miso_io_num = PIN_SPI_MISO,
                                     .sclk_io_num = PIN_SPI_SCLK,
                                     .quadwp_io_num = -1,
                                     .quadhd_io_num = -1,
                                     .data4_io_num = -1,
                                     .data5_io_num = -1,
                                     .data6_io_num = -1,
                                     .data7_io_num = -1,
                                     .max_transfer_sz = 32,  // Min. 32 bytes
                                     .flags = 0,             // SIMPLE mode (1-bit SPI)
                                     .isr_cpu_id = INTR_CPU_ID_AUTO,
                                     .intr_flags = 0};

    spi_host_device_t host = SPI2_HOST;
    esp_err_t ret = spi_bus_initialize(host, &SpiBusConfig, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        LOG_WARN(Logger::Nodes::Main, "SPI init failed: %i", ret);
        return;
    }

    spi_device_interface_config_t ifx9201SG_SpiConfig;
    ifx9201SG_SpiConfig.command_bits = 0U;
    ifx9201SG_SpiConfig.address_bits = 0U;
    ifx9201SG_SpiConfig.dummy_bits = 0U;
    ifx9201SG_SpiConfig.mode = 0U;  // SCK Low at idle + Read on rising edges
    ifx9201SG_SpiConfig.clock_source = spi_clock_source_t::SPI_CLK_SRC_DEFAULT;
    ifx9201SG_SpiConfig.cs_ena_pretrans = 1U;
    ifx9201SG_SpiConfig.cs_ena_posttrans = 1U;
    ifx9201SG_SpiConfig.clock_speed_hz = 1'000'000;
    ifx9201SG_SpiConfig.input_delay_ns = 0;
    ifx9201SG_SpiConfig.spics_io_num = GPIO_NUM_NC;
    ifx9201SG_SpiConfig.queue_size = 1;

    spi_device_handle_t spiDeviceHandle;
    ret = spi_bus_add_device(host, &ifx9201SG_SpiConfig, &spiDeviceHandle);
    if (ret != ESP_OK)
    {
        LOG_WARN(Logger::Nodes::Main, "Device add failed: %i", ret);
        spi_bus_free(host);
        return;
    }

    spi_transaction_t transactionTx{.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
                                    .cmd = 0U,
                                    .addr = 0U,
                                    .length = 8U,
                                    .rxlength = 8U,
                                    .user = nullptr,
                                    .tx_data = {SPI_CMD_RD_REV},
                                    .rx_data = {0U}};

    spi_transaction_t* transactionRx = nullptr;

    IO::DigitalOutput chipSelect(GPIO_NUM_16, IO::eIOState::HIGH_);

    bool cmdQueued = false;
    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        if (!cmdQueued)
        {
            chipSelect.write(IO::eIOState::LOW_);
            LOG_INFO(Logger::Nodes::Main, "%i", spi_device_queue_trans(spiDeviceHandle, &transactionTx, pdTICKS_TO_MS(0U)));
            cmdQueued = true;
        }
        else if (cmdQueued)
        {
            esp_err_t statusCode = spi_device_get_trans_result(spiDeviceHandle, &transactionRx, pdTICKS_TO_MS(0U));
            LOG_INFO(Logger::Nodes::Main, "%i", statusCode);

            if (statusCode == ESP_OK)
            {
                chipSelect.write(IO::eIOState::HIGH_);
                LOG_INFO(Logger::Nodes::Main, "Received data is: %i", transactionRx->rx_data[0]);
                cmdQueued = false;
                delayMicroseconds(4);
            }
        }

        led.update();
    }
}

void loop() {}
