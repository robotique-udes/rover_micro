/**
 * @file can_test.hpp
 * @brief Canbus lib example and integration test
 *
 * @test:
 * 1. Connect the CAN->USB adapter to a linux computer, (make sure to setup your laptop with the steps in
 * rover_document/.../can_setup.md).
 * 2. Write this command: while true; do cansend canRovus 7FF#010000008A42; cansend canRovus 7FF#010101; sleep 0.1; done
 * 3. Upload this code into any device (current pin numbers fit with ArmJ345-REV0).
 * 4. Connect the device canbus connector to your USB->CAN adapter.
 * 5. Open a serial monitor
 * 6. There should be a print with the value sent by step #2: cmd = 69.0F, closed_loop = true;
 * 7. Stop the cmd sent at step #2
 * 8. The LED pattern should change to the watchdog triggered pattern
 * 9. 5s after boot, the device should send a error state message with error=1. You can check this with the `candump canRovus`
 * command in your terminal. Expected output (at ~0.5Hz): "canRovus  7FF   [5]  11 00 01 00 00"
 */

/**
 * @file can_test.hpp
 * @brief Canbus lib example and integration test
 *
 * @test:
 * 1. Connect the CAN->USB adapter to a linux computer, (make sure to setup your laptop with the steps in
 * rover_document/.../can_setup.md).
 * 2. Write this command: while true; do cansend canRovus 7FF#010000008A42; cansend canRovus 7FF#010101; sleep 0.1; done
 * 3. Upload this code into any device (current pin numbers fit with ArmJ345-REV0).
 * 4. Connect the device canbus connector to your USB->CAN adapter.
 * 5. Open a serial monitor
 * 6. There should be a print with the value sent by step #2: cmd = 69.0F, closed_loop = true;
 * 7. Stop the cmd sent at step #2
 * 8. The LED pattern should change to the watchdog triggered pattern
 * 9. 5s after boot, the device should send a error state message with error=1. You can check this with the `candump canRovus`
 * command in your terminal. Expected msg:
 */

#include <Arduino.h>

#include "rover_can2/msgs/test_msg.hpp"
#include "rover_can2/rover_can.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include "rover_lib2/IO/digital_IO.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/LED/led_blinker.hpp"

#include "rover_can2/drivers/driver_esp32.hpp"

constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_4;
constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_9;
constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

class ExampleCanDevice : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, ExampleCanDevice>,
                                                  RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>>
{
  public:
    ExampleCanDevice():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, ExampleCanDevice>,
                          RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>>(
            RoverCan2::Constant::eDeviceId::TEST_DEVICE,
            RoverCan2::SubscriberMember(*this, &ExampleCanDevice::CB_testMsg),
            RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>())

    {
    }

  private:
    void CB_testMsg(const RoverCan2::Msgs::TestMsg& msg_)
    {
        (void)msg_;

        LOG_INFO(Logger::Nodes::Main,
                 "New msg received: msg_.cmd: %f, msg_.closeLoop: %d",
                 msg_.getData().cmd,
                 msg_.getData().closeLoop);

        RoverCan2::Msgs::ErrorState msg;
        msg.data().error = true;
        this->sendMsg<RoverCan2::Msgs::ErrorState>(msg);
    }
};

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft canLed
        = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_CAN_LED), RoverCan2::Constant::LedPatterns::DRIVER_NOT_STARTED);
    RoverCan2::Drivers::DriverESP32 canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    ExampleCanDevice device;
    RoverCan2::Manager canManager(canDriver, device);
    canManager.init();

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);

    OneShotTimer<uint64_t, Time::millis> healthStateSwitchTimer(5'000ULL);
    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        led.update();
        canManager.update();

        if (healthStateSwitchTimer.isReady())
        {
            HealthState::getInstance().setInError();
        }
    }
}

void loop() {}
