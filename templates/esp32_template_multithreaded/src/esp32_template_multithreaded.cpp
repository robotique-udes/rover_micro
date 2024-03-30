#include <Arduino.h>

#if !defined (ESP32)
#error CPU not supported
#endif

#include <std_msgs/msg/int32.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

// =============================================================================
//  This project can be used as a starting template for ESP32 microROS project
//
//  Logging:
//      Since microROS is talking over serial, you can't directly use
//      Serial.print to print. Instead use the LOG() macro ex:
//          LOG(INFO, "some_text %i", some_int);
//
//      To see the log in a terminal, run this cmd in a terminal:
//          ros2 launch rover_helper terminal_logger.launch.py
//
//      If the node is connected with a micro_ros_agent, it will output in the
//      terminal. If, for some reasons, your node isn't connecting, the output
//      will be printed in a terminal at the specified baud in Serial.begin().
//          **TODO**: Add macro to change the LOG macro stream
//
//      The logging level setting can be changed in the platformio.ini file by
//      changing the following entry: '-D LOGGER_LOWEST_LEVEL=10' to the level
//      you want (10, 20, 30, 40, 50)
//
//      Limitations:
//          Not thread safe, don't use in different threads, instead, share a 
//          buffer between both core and LOG from the other core.
//
//  RoverMicroRosLib:
//      You can use this in house library to help writing node faster and safer
//      If you have any question with it you can contact the maintainer in the 
//      lib_rover/library.properties file.
//      Current limitiation can be found in each header files
//
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

    // Start the microRosLoop() in other CPU core to allow maximum performance 
    // in core 1 running setup()
    xTaskCreatePinnedToCore(microRosLoop, "", 8192, NULL, 1, NULL, 0);

    // This a basic stepper control loop with a fixed period of 200*2 = 400us
    // between steps just for the example puposes.
    RoverHelpers::Timer<unsigned long, micros> timer(200ul);
    uint8_t motorStep = HIGH;
    uint8_t PIN_PULSE = 10u;

    for (EVER)
    {
        if (timer.isDone())
        {
            // Switch between LOW and HIGH each loop at period/2
            motorStep = motorStep == HIGH ? LOW : HIGH;
            digitalWrite(PIN_PULSE, motorStep);
        }
    }
}

void microRosLoop(void *pvParameters)
{
    // Use the same transports as set in platformio.ini, wifi isn't supported 
    // yet.
    set_microros_serial_transports(Serial);

    // Create the microROS node. Template parameters are: 
    // <Number of publisher, Number of timers, Number of subscriber>.
    RoverMicroRosLib::Node<1u, 1u, 1u> node(NAME_NS, NAME_NODE);
    // Necessary
    node.init();

    // Link global publisher, timer and subscriber with the node, this will
    // later allow the node to call subscriber and timer callback when those are
    // ready (on new data for example).
    node.addPublisher(&pub);
    node.addTimer(&timer);
    node.addSubscriber(&sub);

    for (EVER)
    {
        // This handle the connection with the ROS network and calls ready 
        // callbacks.
        node.spinSome(RCL_MS_TO_NS(0UL));
    }
}

// This function is called each at the set timer period
void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
{
    // Create instance of msg and adds data to the msg which is then sent on the 
    // publisher's topic.
    std_msgs__msg__Int32 msg;
    msg.data = 88L;
    pub.publish(&msg);
}

// This function is called each time new data is available on the 
// /pub_test_topic topic.
void cbSubscriber(const void *msg_)
{
    // Cast the void pointer into the correct message type for the topic. This 
    // is necessary to access the data in the correct format. It then logs the
    // msg data into the terminal logger. 
    // (ros2 launch rover_helper terminal_logger.launch.py)
    
    const std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32 *)msg_;
    LOG(INFO, "Received: %i", msg->data);
}

// Do not use when using FreeRTOS.
void loop() {}
