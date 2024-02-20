#if !defined (ESP32)
#error CPU not supported
#endif

#include <Arduino.h>

#include <std_msgs/msg/int32.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "helpers/helpers.hpp"

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    MovingAverage<float, 10> avg(5.0f);

    RoverMicroRosLib::Publisher pub(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "Test");
    RoverMicroRosLib::Node<1u, 0u, 0u> node(NAME_NS, NAME_NODE);
    Timer<unsigned long, millis> timerTest(1000ul);
    node.init();
    node.addPublisher(&pub);
    
    for (EVER)
    {
        node.spinSome(RCL_MS_TO_NS(0UL));

        if (timerTest.isDone())
        {
            LOG(INFO, "%f", avg.addValue(10.0f));
            std_msgs__msg__Int32 msg;
            msg.data = (int32_t)round(avg.getAverage());
            pub.publish(&msg);
        }
    }
}

// Do not use when using FreeRTOS.
void loop() {}
