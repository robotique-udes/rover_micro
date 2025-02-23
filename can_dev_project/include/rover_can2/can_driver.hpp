#ifndef CAN_DRIVER
#define CAN_DRIVER

#if defined(ARDUINO_ESP32S3_DEV)
#include "driver/gpio.h"
#include "driver/twai.h"
#endif  // defined(ARDUINO_ESP32S3_DEV)

namespace RoverCan2
{

#if defined(ARDUINO_ESP32S3_DEV)

    class CanDriver
    {
        static constexpr twai_timing_config_t CAN_SPEED_CONFIG = TWAI_TIMING_CONFIG_1MBITS();
        static constexpr twai_mode_t TWAI_MODE = TWAI_MODE_NORMAL;
        static constexpr twai_filter_config_t TWAI_ID_FILTER = TWAI_FILTER_CONFIG_ACCEPT_ALL();

      public:
        CanDriver(gpio_num_t ioRx_, gpio_num_t ioTx_): _ioRx(ioRx_), _ioTx(ioTx_) {}

        void init(void)
        {
            twai_general_config_t genConfig = TWAI_GENERAL_CONFIG_DEFAULT(_ioRx, _ioTx, TWAI_MODE);
            twai_timing_config_t timingConfig = CAN_SPEED_CONFIG;
            twai_filter_config_t IDFilterConfig = TWAI_ID_FILTER;

            if (twai_driver_install(&genConfig, &timingConfig, &TWAI_ID_FILTER) == ESP_OK)
            {
                printf("Driver installed\n");
            }
            else
            {
                printf("Failed to install driver\n");
                return;
            }
        }

      private:
        const gpio_num_t _ioRx;
        const gpio_num_t _ioTx;
    };

#endif  // defined(ARDUINO_ESP32S3_DEV)

}  // namespace RoverCan2

#endif  // CAN_DRIVER
