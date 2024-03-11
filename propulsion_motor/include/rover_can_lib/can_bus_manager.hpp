#ifndef __CAN_BUS_MANAGER_HPP__
#define __CAN_BUS_MANAGER_HPP__

#if defined(ESP32)

#include "driver/twai.h"

#include "helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"

namespace RoverCanLib
{
    class CanBusManager
    {
    public:
        enum eCanBusStatus : uint8_t
        {
            notInit,
            idle,
            running,
            warning,
            ERROR_DELIMITER,
            watchdogError,
            error
        };

        CanBusManager(uint16_t deviceId_,
                      gpio_num_t txPin_,
                      gpio_num_t rxPin_,
                      void (*canMsgCallbackFunc_)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_),
                      gpio_num_t pinStatusLED_ = GPIO_NUM_NC,
                      twai_mode_t nodeMode_ = twai_mode_t::TWAI_MODE_NORMAL,
                      twai_timing_config_t configSpeed_ = TWAI_TIMING_CONFIG_1MBITS());
        ~CanBusManager();
        void init();
        void sendMsg(twai_message_t *msg_);
        void sendMsg(uint32_t id_, uint8_t *data_, uint8_t dataLenght_, bool validateIntegrity_ = false);
        void readMsg(OUT twai_message_t *msg);
        void update(void);

        void updateLedStatus(void);
        void updateHeartbeat(void);
        void updateCallbackMsg(void);
        void updateWatchdog(void);
        void setWarningFlag(void);
        void resetWatchDog(void);
        bool isOk(void);

    private:
        uint16_t _id;

        bool _withLed = false;
        LedBlinker _statusLed;

        eCanBusStatus _lastCanBusState;
        eCanBusStatus _canBusState = eCanBusStatus::notInit;
        bool _flagWarning = false;

        Timer<unsigned long, millis> _timerHeartbeat;
        Timer<unsigned long, millis> _timerWatchdog = Timer<unsigned long, millis>(RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS);

        bool _watchDogAlive = false;

        void (*_canMsgCallbackFunc)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_) = NULL;
    };
}

#endif // !defined(ESP32)
#endif // __CAN_BUS_MANAGER_HPP__
