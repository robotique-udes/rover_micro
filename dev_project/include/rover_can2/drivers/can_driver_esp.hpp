#ifndef CAN_DRIVER_ESP_HPP
#define CAN_DRIVER_ESP_HPP

#include "rover_can2/drivers/can_driver.hpp"

#if defined(ARDUINO_ESP32S3_DEV)
#include "rover_can2/can_msg.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "optional"

#include "rover_lib2/helpers/circular_buffer.hpp"

#include "driver/gpio.h"
#include "driver/twai.h"
#endif  // defined(ARDUINO_ESP32S3_DEV)

DEFINE_LOG_NODE(CanDriverESP32, Logger::eNodeState::ON);

namespace RoverCan2
{
#if defined(ARDUINO_ESP32S3_DEV)
    class CanDriverESP : public CanDriver
    {
      public:
        static constexpr twai_timing_config_t CAN_SPEED_CONFIG = TWAI_TIMING_CONFIG_1MBITS();
        static constexpr twai_mode_t TWAI_MODE = TWAI_MODE_NORMAL;
        static constexpr twai_filter_config_t TWAI_ID_FILTER = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        static constexpr TickType_t MESSAGE_RECV_TIMEOUT = 0U;  // Doesn't wait if no msg available

        static constexpr size_t CAN_DATA_LENGTH = TWAI_FRAME_MAX_DLC;
        static_assert(Constant::CAN_MAX_DATA_LENGTH == CAN_DATA_LENGTH);

        enum eState : size_t
        {
            UNINSTALLED,
            INSTALLED,
            RUNNING,
        };

      public:
        CanDriverESP(gpio_num_t ioRx_, gpio_num_t ioTx_):
            _ioRx(ioRx_),
            _ioTx(ioTx_),
            _state(eState::UNINSTALLED)
        {
        }

        void init(void) override
        {
            this->installDriver();
            this->startDriver();
        }

        void update(void) override
        {
            switch (_state)
            {
                case eState::UNINSTALLED:
                    this->installDriver();
                    [[fallthrough]];
                case eState::INSTALLED:
                    this->startDriver();
                    [[fallthrough]];
                case eState::RUNNING:
                    this->processNewMessage();
                    break;
            }
        }

        size_t getAvailableMessagesNb(void) const
        {
            return _msgBuffer.size();
        }

        std::optional<CanMsg> getMsg(void)
        {
            return _msgBuffer.getValue();
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
                    LOG_DEBUG(Logger::Nodes::CanDriverESP32, "Twai driver installed successfully");
                    _state = eState::INSTALLED;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::CanDriverESP32, "Can't install twai driver in current state (%u)", TO_UNDERLYING(_state));
                    break;
                case ESP_ERR_INVALID_ARG:
                    ASSERT("Can't install twai driver with specified arguments");
                    break;
                case ESP_ERR_NO_MEM:
                    ASSERT("Can't install twai driver... no more memory");
                    break;
                default:
                    ASSERT("Can't install twai driver... Unknown error %d", successCode);
                    break;
            }
        }

        void startDriver(void)
        {
            esp_err_t successCode = twai_start();
            switch (successCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::CanDriverESP32, "Twai driver started successfully");
                    _state = eState::RUNNING;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::CanDriverESP32, "Can't install twai driver in current state %u", TO_UNDERLYING(_state));
                    break;
                default:
                    ASSERT("Can't start twai driver... Unknown error %d", successCode);
                    break;
            }
        }

        void processNewMessage(void)
        {
            twai_message_t message;

            esp_err_t statusCode = twai_receive(&message, MESSAGE_RECV_TIMEOUT);
            switch (statusCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::CanDriverESP32, "New message received");
                    break;
                case ESP_ERR_TIMEOUT:
                    return;
                case ESP_ERR_INVALID_STATE:
                    LOG_ERROR(Logger::Nodes::CanDriverESP32, "Can Driver has fallen into an invalid state, trying to recover...");
                    _state = eState::UNINSTALLED;
                    return;
                case ESP_ERR_INVALID_ARG:
                    ASSERT("Invalid arguments, implementation mistake");
                    return;
            }

            if (!twaiMsgValid(message))
            {
                return;
            }

            CanMsg msg(message);
            if (msg.msgID == RoverCan2::Constant::eMsgId::INVALID)
            {
                LOG_WARN(Logger::Nodes::CanDriverESP32, "Received invalid message dropping");
                return;
            }

            decltype(_msgBuffer)::eErrorCode status = _msgBuffer.addValue(msg);
            switch (status)
            {
                case decltype(_msgBuffer)::eErrorCode::SUCCESS:
                    break;
                case decltype(_msgBuffer)::eErrorCode::SUCCESS_DATA_LOSS:
                    LOG_WARN(Logger::Nodes::CanDriverESP32, "Msg buffer full, losing data");
                    break;
                case decltype(_msgBuffer)::eErrorCode::ERROR:
                    LOG_WARN(Logger::Nodes::CanDriverESP32, "Unknown error");
                    break;
            }
        }

        bool twaiMsgValid(const twai_message_t& msg_)
        {
            if (msg_.extd)
            {
                LOG_WARN(Logger::Nodes::CanDriverESP32, "Received unsupported extended frame, skipping...");
                return false;
            }
            else if (msg_.rtr)
            {
                LOG_DEBUG(Logger::Nodes::CanDriverESP32, "Received remote frame, skipping...");
                return false;
            }
            else if (msg_.dlc_non_comp)
            {
                LOG_WARN(Logger::Nodes::CanDriverESP32, "Received too long data frame, skipping...");
                return false;
            }

            return true;
        }

        const gpio_num_t _ioRx;
        const gpio_num_t _ioTx;

        eState _state;
        CircularBuffer<CanMsg, 10UL> _msgBuffer;
    };
#endif  // defined(ARDUINO_ESP32S3_DEV)
}  // namespace RoverCan2

#endif  // CAN_DRIVER_ESP_HPP
