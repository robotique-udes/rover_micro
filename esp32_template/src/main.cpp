#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "helpers/log.h"
#include "helpers/microROS_manager.h"

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
//          ros2 launch rover_helper terminal_logger.py
//
//      If the node is connected with a micro_ros_agent, it will output in the
//      terminal. If, for some reasons, your node isn't connecting, the output
//      will be printed in a terminal at 115200 baud.
//
//      The logging level setting can be changed in the platformio.ini file by
//      changing the following entry: '-D LOGGER_LOWEST_LEVEL=10' to the level
//      you want (10, 20, 30, 40, 50)
//
//  Developpement guidelines:
//      Write all function that can be reused in other projects in the
//      "lib_rover" library located in rover/rover_micro_ros_projects/lib_rover
//      the library is included in the platformio.ini file
//
//  MicroROSManagerCustom:
//      This class is a helper that handles all necessary connection with a
//      micro_ros_agent for you.
//
// =============================================================================

// Function forward declarations
// void cbTimer(rcl_timer_t *timer, int64_t last_call_time);
// void cbSubscriber(const void *msg_);

// Global objects
MicroRosPublisher pub = MicroRosPublisher(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pub_test_topic");
// rcl_subscription_t sub;
// int32_t counter;

void setup()
{
    // Open USB serial port and give it to micro_ros for communication with
    // micro_ros_agent.
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    MicroROSManager<1u, 0u> rosManager(NAME_NS, NAME_NODE);
    rosManager.init();
    rosManager.addPublisher(&pub);

    for (EVER)
    {
        rosManager.spinSome(0UL);
    }
}

// // This function is called each at the set timer frequency
// void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
// {
//     (void)last_call_time;
//     if (timer != NULL)
//     {
//         std_msgs__msg__Int32 msg;
//         msg.data = counter++;
//         REMOVE_WARN_UNUSED(rcl_publish(&pub, &msg, NULL));
//         LOG(INFO, "Sent %i", msg.data);
//     }
// }

// void cbSubscriber(const void *msg_)
// {
//     const std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32*)msg_;

//     LOG(INFO, "Received: %i", msg->data);
// }
