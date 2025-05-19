#include <Arduino.h>
#include <GNSSManager.hpp>
#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

// Configure UART pins
constexpr gpio_num_t PIN_UART_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_UART_RX = GPIO_NUM_13;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_4;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_5;
constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_2;

#define UART_BAUD_RATE 115200
DEFINE_LOG_NODE(Main, Logger::eNodeState::OFF);


class CanGNSS : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>, 
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>, 
                                         public RoverObject<CanGNSS>
{
  public:
    CanGNSS():
        RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                          RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>, 
                          RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>(
                              RoverCan2::Constant::eDeviceId::GNSS,
                              RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>(),
                              RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>(), 
                              RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>()),
                          updateTimer(50)
    {
    }

    void _init(void){}

    void _update(void)
    {
      if (!updateTimer.isReady()) 
      {
        return;
      }
      this->sendMsg(posMsg);
      this->sendMsg(headingMsg);
      this->sendMsg(infoMsg);
    }

    void set(float latitude_, float longitude_, float headingDeg_, int fixQuality_, int satellites_)
    {
      posMsg.data().latitude = latitude_;
      posMsg.data().longitude = longitude_;

      headingMsg.data().headingDeg = headingDeg_;
      
      infoMsg.data().fixQuality = fixQuality_;
      infoMsg.data().satelliteCount = satellites_;
    }
  
  private:
    RoverCan2::Msgs::FixPosition posMsg;
    RoverCan2::Msgs::FixHeading headingMsg;
    RoverCan2::Msgs::FixInfo infoMsg;
    RoverCan2::Publisher<RoverCan2::Msgs::FixPosition> _fixPosPublisher;
    RoverCan2::Publisher<RoverCan2::Msgs::FixHeading> _fixHeadingPublisher;
    RoverCan2::Publisher<RoverCan2::Msgs::FixInfo> _fixInfoPublisher;
    LoopTimer<uint64_t, Time::millis> updateTimer;
};


void setup()
{
  LED::LedBlinkerSoft canLed = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_LED_CAN), 
                               LED::BlinkPatterns::ON);
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
    sGNSSData data = gnss.getData();

    device.set(data.latitude, data.longitude, data.headingDeg, data.fixQuality, data.satellites);
    
    if (data.hasValidFix()) 
    {
      LOG_INFO(Logger::Nodes::Main, "Lat: %.6f, Lon: %.6f, Heading: %f deg, Quality: %d, Satellites: %d\n",
               data.latitude, data.longitude, data.headingDeg, data.fixQuality, data.satellites);
    } else 
    {
      LOG_INFO(Logger::Nodes::Main, "Waiting for a valid fix...");
    }    
    
    canManager.update();
  }
}


void loop()
{
}
