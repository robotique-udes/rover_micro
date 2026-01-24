
#include "config.hpp"
//#include "J1Device.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::OFF);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft statusLed(IO::DigitalOutput(PIN_CAN_LED), LED::BlinkPatterns::HEARTBEAT);
    statusLed.init();

    J1Device j1;
    j1.init();

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(PIN_CAN_LED), LED::BlinkPatterns::HEARTBEAT);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    RoverCan2::ManagerSlave canManager(canDriver, j1.getJ1Device());
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "J1 Init done, starting main loop!");
    for (EVER)
    {
        statusLed.update();
        j1.update();
        canManager.update();
    }
}

void loop()
{
    // Don't use
}