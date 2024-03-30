#if !defined (ESP32)
#error CPU not supported
#endif

#include <Arduino.h>

#include <std_msgs/msg/int32.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

// =============================================================================
//  This project can be used as a starting template for ESP32 microROS project
//  A commented version of this file is also available as template
// =============================================================================

// Function forward declarations
void microRosLoop(void *pvParameters);
void cbTimer(rcl_timer_t *timer, int64_t last_call_time);
void cbSubscriber(const void *msg_);

// Global objects
RoverMicroRosLib::Publisher pub = RoverMicroRosLib::Publisher(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pub_test_topic");
RoverMicroRosLib::Timer timer = RoverMicroRosLib::Timer(RCL_MS_TO_NS(100), cbTimer);
RoverMicroRosLib::Subscriber sub = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pub_test_topic", cbSubscriber);

void setup()
{
    Serial.begin(115200);
    xTaskCreatePinnedToCore(microRosLoop, "", 8192, NULL, 1, NULL, 0);

    RoverHelpers::Timer<unsigned long, micros> timer(200ul);
    uint8_t motorStep = HIGH;
    uint8_t PIN_PULSE = 10u;

    for (EVER)
    {
        if (timer.isDone())
        {
            motorStep = motorStep == HIGH ? LOW : HIGH;
            digitalWrite(PIN_PULSE, motorStep);
        }
    }
}

void microRosLoop(void *pvParameters)
{
    set_microros_serial_transports(Serial);

    RoverMicroRosLib::Node<1u, 1u, 1u> node(NAME_NS, NAME_NODE);
    node.init();
    node.addPublisher(&pub);
    node.addTimer(&timer);
    node.addSubscriber(&sub);

    for (EVER)
    {
        node.spinSome(RCL_MS_TO_NS(0UL));
    }
}

void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
{
    std_msgs__msg__Int32 msg;
    msg.data = 88L;
    pub.publish(&msg);
}

void cbSubscriber(const void *msg_)
{
    const std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32 *)msg_;
    LOG(INFO, "Received: %i", msg->data);
}

void loop() {}
