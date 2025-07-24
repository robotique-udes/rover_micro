#include <Arduino.h>
#include "GNSSManager.hpp"
#include "CanGNSS.hpp"

// Configure UART pins
constexpr gpio_num_t PIN_UART_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_UART_RX = GPIO_NUM_13;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_4;
constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_2;

constexpr uint32_t UART_BAUD_RATE = 115200UL;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup()
{
    Serial.begin(115200);
    LED::LedBlinkerSoft canLed = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_LED_CAN), LED::BlinkPatterns::ON);
    RoverCan2::Drivers::DriverESP32 canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    CanGNSS device;
    RoverCan2::ManagerSlave canManager(canDriver, device);
    canManager.init();

    // Initialize HardwareSerial on UART2
    GNSSManager gnss(Serial2);
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

    for (EVER)
    {
        gnss.update();
        device._update();
        sGNSSData data = gnss.getData();

        device.set(data);

        if (data.hasValidFix())
        {
            LOG_INFO(Logger::Nodes::Main,
                     "Lat: %.6f, Lon: %.6f, Heading: %f deg, Fix Quality: %d, Heading Quality: %d, Satellites: %d\n",
                     data.latitude,
                     data.longitude,
                     data.headingDeg,
                     data.fixQuality,
                     data.headingQuality,
                     data.satellites);
        }
        else
        {
            sGNSSData invalidData;  // set all values to default (zeros everywhere)
            device.set(invalidData);
            LOG_INFO(Logger::Nodes::Main, "Waiting for a valid fix...");
        }

        canManager.update();
    }
}

void loop() {}
