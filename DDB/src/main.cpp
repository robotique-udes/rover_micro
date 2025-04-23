#include <Arduino.h>

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "DDB.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

constexpr gpio_num_t PIN_LED_USER = GPIO_NUM_2;
constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_1;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_48;

void setup()
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    DDB ddb;
    ddb.init();

    LED::LedBlinkerSoft ledHeartbeat(IO::DigitalOutput(PIN_LED_USER), LED::BlinkPatterns::HEARTBEAT);
    ledHeartbeat.init();

    LED::LedBlinkerSoft ledCan(IO::DigitalOutput(PIN_LED_CAN), LED::BlinkPatterns::ON);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(PIN_CAN_RX, PIN_CAN_TX, &ledCan);
    RoverCan2::Manager canManager(canDriver, ddb);
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "Init done starting");
    for (EVER)
    {
        ledHeartbeat.update();
        canManager.update();
        ddb.update();
    }
}

void loop() {}
