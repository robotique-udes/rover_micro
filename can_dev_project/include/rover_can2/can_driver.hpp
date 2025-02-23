#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

#if defined(ARDUINO_ESP32S3_DEV)
#include "driver/gpio.h"
#include "driver/twai.h"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/log.hpp"
#endif  // defined(ARDUINO_ESP32S3_DEV)

DEFINE_LOG_NODE(CanDriver, Logger::eNodeState::ON);

namespace
{
    constexpr Logger::Nodes::Node LOGGER = Logger::Nodes::CanDriver;
}

namespace RoverCan2
{

#if defined(ARDUINO_ESP32S3_DEV)

    class CanDriver
    {
        static constexpr twai_timing_config_t CAN_SPEED_CONFIG = TWAI_TIMING_CONFIG_1MBITS();
        static constexpr twai_mode_t TWAI_MODE = TWAI_MODE_NORMAL;
        static constexpr twai_filter_config_t TWAI_ID_FILTER = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        static constexpr TickType_t MESSAGE_RECV_TIMEOUT = 0U;  // Don't wait

        enum eState : size_t
        {
            UNINSTALLED,
            INSTALLED,
            RUNNING,
        };

      public:
        CanDriver(gpio_num_t ioRx_, gpio_num_t ioTx_): _ioRx(ioRx_), _ioTx(ioTx_), _state(eState::UNINSTALLED) {}

        void init(void)
        {
            this->installDriver();
            this->startDriver();
        }

        void update(void)
        {
            switch (_state)
            {
                case eState::UNINSTALLED: this->installDriver(); [[fallthrough]];
                case eState::INSTALLED: this->startDriver(); [[fallthrough]];
                case eState::RUNNING: this->processNewMessage(); break;
            }
        }

      private:
        void installDriver(void)
        {
            twai_general_config_t genConfig = TWAI_GENERAL_CONFIG_DEFAULT(_ioRx, _ioTx, TWAI_MODE);
            twai_timing_config_t timingConfig = CAN_SPEED_CONFIG;
            twai_filter_config_t IDFilterConfig = TWAI_ID_FILTER;

            esp_err_t successCode = twai_driver_install(&genConfig, &timingConfig, &IDFilterConfig);
            switch (successCode)
            {
                case ESP_OK:
                    LOG::DEBUG(LOGGER, "Twai driver installed successfully");
                    _state = eState::INSTALLED;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG::WARN(LOGGER, "Can't install twai driver in current state (%u)", TO_UNDERLYING(_state));
                    break;
                case ESP_ERR_INVALID_ARG: ASSERT("Can't install twai driver with specified arguments"); break;
                case ESP_ERR_NO_MEM: ASSERT("Can't install twai driver... no more memory"); break;
                default: ASSERT("Can't install twai driver... Unknown error %d", successCode); break;
            }
        }

        void startDriver(void)
        {
            esp_err_t successCode = twai_start();
            switch (successCode)
            {
                case ESP_OK:
                    LOG::DEBUG(LOGGER, "Twai driver started successfully");
                    _state = eState::RUNNING;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG::WARN(LOGGER, "Can't install twai driver in current state %u", TO_UNDERLYING(_state));
                    break;
                default: ASSERT("Can't start twai driver... Unknown error %d", successCode); break;
            }
        }

        void processNewMessage(void)
        {
            twai_message_t message;

            esp_err_t statusCode = twai_receive(&message, MESSAGE_RECV_TIMEOUT);
            switch (statusCode)
            {
                case ESP_OK: LOG::DEBUG(LOGGER, "New message received"); break;
                case ESP_ERR_TIMEOUT: LOG::DEBUG(LOGGER, "No message available, skipping"); return;
                case ESP_ERR_INVALID_ARG: ASSERT("Invalid arguments, implementation mistake"); break;
                case ESP_ERR_INVALID_STATE:
                    LOG::ERROR(LOGGER, "Can Driver has fallen into an invalid state, trying to recover...");
                    _state = eState::UNINSTALLED;
                    break;
            }

            //     if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
            // {
            //     printf("Message received\n");
            // }
            // else
            // {
            //     printf("Failed to receive message\n");
            //     return;
            // }

            // // Process received message
            // if (message.extd)
            // {
            //     printf("Message is in Extended Format\n");
            // }
            // else
            // {
            //     printf("Message is in Standard Format\n");
            // }
            // printf("ID is %d\n", message.identifier);
            // if (!(message.rtr))
            // {
            //     for (int i = 0; i < message.data_length_code; i++)
            //     {
            //         printf("Data byte %d = %d\n", i, message.data[i]);
            //     }
            // }
        }

        const gpio_num_t _ioRx;
        const gpio_num_t _ioTx;

        eState _state;
    };

#endif  // defined(ARDUINO_ESP32S3_DEV)

}  // namespace RoverCan2

#endif  // CAN_DRIVER_HPP
