#include "config.hpp"
#include "gas_sensor.hpp"

#include <rover_lib2/helpers/log.hpp>
#include <rover_lib2/LED/led_blinker.hpp>
#include <rover_can2/rover_can2.hpp>
#include <Arduino.h>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);

#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    GasSensor gasSensor(Wire1);
    gasSensor.init();

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(LED_BLTN), LED::BlinkPatterns::ON);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(CAN_TX, CAN_RX, &canLed);

    RoverCan2::ManagerSlave canManager(canDriver, gasSensor.getCanDevice());
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "gasSensor Init done, starting main loop!");
    for (EVER)
    {
        canManager.update();
        gasSensor.update();
    }
}

void loop()
{
    // Don't use
}
