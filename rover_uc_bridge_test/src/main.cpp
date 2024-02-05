#include <Arduino.h>

#include "rover_ros_serial/base_objects.hpp"
#include "rover_ros_serial/logger.hpp"
#include "helpers/timer.h"

#include "helpers/macros.h"
#include "helpers/log.h"

void mainLoop(void *pvParameters);

class Heartbeat : protected RoverRosSerial::Msg
{
public:
    Heartbeat(uint8_t frequency_ = 2u, HardwareSerial *serial_ = &Serial) : Msg(serial_)
    {
        _frequency = frequency_;
        _timerHeartbeatSender.init(static_cast<unsigned long>(round(1000.0f/static_cast<float>(frequency_))));

        uHeader.header.type = RoverRosSerial::Constant::eHeaderType::heartbeat;
        uHeader.header.length = 0u; // Sending Empty msgs
    }
    ~Heartbeat();

    void update()
    {
        if (_timerHeartbeatSender.done())
        {
            LOG(INFO, "Sending heartbeat signal");
            this->sendMsg();
        }
    }

private:
    uint8_t _frequency;
    TimerMillis _timerHeartbeatSender;

    uint8_t *getSerializedData(void) { return NULL; };
    uint8_t getSerializedDataSize(void) { return 0; };
};

void setup()
{
    Serial.begin(115200, SERIAL_8N1);

    // Start taskCommunication on the second core
    xTaskCreatePinnedToCore(mainLoop, NULL, 4096, NULL, 1, NULL, 0);
}

void mainLoop(void *pvParameters)
{
    LOG(eLoggerLevel::WARN, "Init...");
    Heartbeat hearbeat(4u, &Serial);

    for (EVER)
    {
        // LOG(eLoggerLevel::INFO, "Looping");
        hearbeat.update();
    }
}
