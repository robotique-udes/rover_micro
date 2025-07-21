#include <Arduino.h>
#include "GNSSManager.hpp"
#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

// Configure UART pins
constexpr gpio_num_t PIN_UART_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_UART_RX = GPIO_NUM_13;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_4;
constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_2;

constexpr uint32_t UART_BAUD_RATE = 115200;
constexpr uint32_t PUBLISH_PERIOD_FAST_MS = 50;
constexpr uint32_t PUBLISH_PERIOD_SLOW_MS = 1000;
constexpr uint8_t GNSS_UART_PORT = 2;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

class CanGNSS : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>,
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>
{
  public:
    CanGNSS():
        RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                          RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>,
                          RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>(RoverCan2::Constant::eDeviceId::GNSS,
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>(),
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>(),
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>()),
        updateTimerFast(PUBLISH_PERIOD_FAST_MS),
        updateTimerSlow(PUBLISH_PERIOD_SLOW_MS)
    {
    }

    void _init(void) {}

    void _update(void)
    {
        if (updateTimerFast.isReady())
        {
            this->sendMsg(posMsg);
            this->sendMsg(headingMsg);
        }
        if (updateTimerSlow.isReady())
        {
            this->sendMsg(infoMsg);
        }
    }

    void set(float latitude_,
             float longitude_,
             float headingDeg_,
             Constants::GGAQuality fixQuality_,
             Constants::UniHeadingQuality headingQuality_,
             int satellites_)
    {
        posMsg.data().latitude = latitude_;
        posMsg.data().longitude = longitude_;

        headingMsg.data().headingDeg = headingDeg_;

        infoMsg.data().fixQuality = fixQuality_;
        infoMsg.data().headingQuality = headingQuality_;
        infoMsg.data().satelliteCount = satellites_;
    }

  private:
    RoverCan2::Msgs::FixPosition posMsg;
    RoverCan2::Msgs::FixHeading headingMsg;
    RoverCan2::Msgs::FixInfo infoMsg;
    LoopTimer<uint64_t, Time::millis> updateTimerFast;
    LoopTimer<uint64_t, Time::millis> updateTimerSlow;
};

void setup()
{
    Serial.begin(115200);
    LED::LedBlinkerSoft canLed = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_LED_CAN), LED::BlinkPatterns::ON);
    RoverCan2::Drivers::DriverESP32 canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    CanGNSS device;
    RoverCan2::ManagerSlave canManager(canDriver, device);
    canManager.init();

    // Initialize HardwareSerial on UART2
    HardwareSerial GNSSSerial(2);
    GNSSManager gnss(GNSSSerial);
    GNSSSerial.begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

    for (EVER)
    {
        gnss.update();
        device._update();
        sGNSSData data = gnss.getData();

        device.set(data.latitude, data.longitude, data.headingDeg, data.fixQuality, data.headingQuality, data.satellites);

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
            device.set(0.0f, 0.0f, 0UL, Constants::GGAQuality::UNKNOWN, Constants::UniHeadingQuality::NO_HEADING, 0.0f);
            LOG_INFO(Logger::Nodes::Main, "Waiting for a valid fix...");
        }

        canManager.update();
    }
}

void loop() {}
