#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <rover_msgs/msg/antenna_cmd.h>

#include "helpers/log.h"
#include "helpers/timer.h"
#include "helpers/microROS_manager.h"

#define NAME_NS "/esp32_antenna_ns"
#define NAME_NODE "esp32_antenna"

#define EN 34
#define DIR 26
#define PUL 25
#define STEPS_PER_ROUND 1600

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
bool createEntities();
void destroyEntities();
void cbTimer(rcl_timer_t *timer, int64_t last_call_time);
void cbGoal(const void *msg_);
void tickStepper(uint32_t freq);

// Global objects
MicroROSManagerCustom rosManager = MicroROSManagerCustom(createEntities, destroyEntities, true);
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pub;
rcl_subscription_t sub_goal;
int32_t counter;
TimerMicros timer_stepper;


void setup()
{
 
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    pinMode(EN, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(PUL, OUTPUT);

    digitalWrite(EN, LOW);
    digitalWrite(DIR, HIGH);
    //MicroROSManagerCustom rosManager(createEntities, destroyEntities, true);
    rosManager.init();

    // for (EVER)
    // {
    //     // digitalWrite(LED_BUILTIN, HIGH);
    //     // This check if any callback are ready (new msg received or a done 
    //     // timer). It must be call quite often, each loop is a rule of thumb. 
    //     // This is also where MicroROSManagerCustom check connection state and 
    //     // acts accordingly

    //     rosManager.spinSome(&executor, 0UL);
    // }
}

void loop() {

    // digitalWrite(PUL,HIGH);
    // delayMicroseconds(200);
    // digitalWrite(PUL,LOW);
    // tickStepper(200);
    rosManager.spinSome(&executor, 0UL);
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));
    RCLC_RET_ON_ERR(rclc_node_init_default(&node, NAME_NODE, NAME_NS, &support));

    Logger.createLogger(&node, NAME_NODE, NAME_NS);

    // create timer (optional)
    // The passed function is called each time each time the timer is ready at
    // the "rosManager.spinSome(&executor, 0UL);" call")
    RCLC_RET_ON_ERR(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), cbTimer));

    RCLC_RET_ON_ERR(rclc_subscription_init_default(&sub_goal, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, AntennaCmd), "/base/antenna/cmd/out/goal"))

    // create executor (required)
    executor = rclc_executor_get_zero_initialized_executor();

    // Set the number of handle (here 2) is the total number of subscriptions,
    // timers, services, clients and guard conditions. Do not include the number
    // of nodes and publishers. (required)
    RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add this line for each timer (links the timer with the executor)
    RCLC_RET_ON_ERR(rclc_executor_add_timer(&executor, &timer));

    // Add these lines for each subscriber (links the sub with the executor), 
    // You also need to create a temporary msg of the sub msg type and passed 
    // the pointer to the executor for memory allocation
    rover_msgs__msg__AntennaCmd msg;
    RCLC_RET_ON_ERR(rclc_executor_add_subscription(&executor, &sub_goal, &msg, &cbGoal, ON_NEW_DATA));

    return true;
}

void destroyEntities()
{
    // necessary
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // "Fini" each element created inside the createEntities
    Logger.destroyLogger(&node);
    REMOVE_WARN_UNUSED(rcl_publisher_fini(&pub, &node));
    REMOVE_WARN_UNUSED(rcl_timer_fini(&timer));
    REMOVE_WARN_UNUSED(rcl_subscription_fini(&sub_goal, &node));
    REMOVE_WARN_UNUSED(rclc_executor_fini(&executor));
    REMOVE_WARN_UNUSED(rcl_node_fini(&node));
    REMOVE_WARN_UNUSED(rclc_support_fini(&support));
}

// This function is called each at the set timer frequency
void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        std_msgs__msg__Int32 msg;
        msg.data = counter++;
        REMOVE_WARN_UNUSED(rcl_publish(&pub, &msg, NULL));
        LOG(INFO, "Sent %i", msg.data);
    }
}

void cbGoal(const void *msg_)
{
    const rover_msgs__msg__AntennaCmd *msg = (rover_msgs__msg__AntennaCmd*)msg_; 
    //LOG(INFO, "Received: %f", msg->speed);
    //tickStepper(1);
    // if(msg->speed != 0.0)
    // {
    //     float freq = 360 / (STEPS_PER_ROUND * abs(msg->speed)) * 1e-6;
    //     //LOG(INFO, "Received: %f", freq);        
    // }
    
    tickStepper(200);

}

void tickStepper(uint32_t freq){
    //LOG(INFO, "In tickStepper");
    digitalWrite(PUL,HIGH);
    digitalWrite(PUL,LOW);
    timer_stepper.init(freq);
}