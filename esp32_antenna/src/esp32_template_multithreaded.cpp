#include <Arduino.h>

#if !defined(ESP32)
#error CPU not supported
#endif

#include <std_msgs/msg/int32.h>
#include "rover_msgs/msg/antenna_cmd.h"
#include <std_msgs/msg/empty.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "helpers/helpers.hpp"

#include <semaphore>

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

#define PUL 27
#define DIR 26
#define EN 25

#define PI 3.1415
#define MICRO_STEPS 800
#define RATIO_MOTOR 99.51

#define HEARTBEAT_HZ 4

// 800 micro-step/tour * ratio de la gearbox

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
void cbGoal(const void *msg_);
void cbSubHeartbeat(const void *msg_);

// Global objects
RoverMicroRosLib::Subscriber subGoal = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, AntennaCmd), "/base/antenna/cmd/out/goal", cbGoal);
RoverMicroRosLib::Subscriber subHeartbeat = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/base/base_heartbeat", cbSubHeartbeat);
Timer<unsigned long, micros> timer(200ul);
Timer<unsigned long, millis> timer_heartbeat(500);
bool root_heartbeat_state = false;
bool state_heartbeat = false;
bool motor_status = false;
bool stepper_direction;

// Creating mutex
SemaphoreHandle_t xSemaphore = NULL;

void setup()
{
    Serial.begin(115200);
    pinMode(PUL, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(DIR, OUTPUT);

    xTaskCreatePinnedToCore(microRosLoop, "", 8192, NULL, 1, NULL, 0);

    xSemaphore = xSemaphoreCreateMutex();

    uint8_t motorStep = HIGH;
    uint8_t PIN_PULSE = 10u;

    digitalWrite(EN, LOW);
    digitalWrite(DIR, HIGH);

    for (EVER)
    {
        if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
        {
            state_heartbeat = true;
        }
        else
        {
            state_heartbeat = false;
        }

        if (timer_heartbeat.isDone())
        {
            root_heartbeat_state = false;
        }

        if (root_heartbeat_state)
        {
            if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE)
            {
                motorStep = motorStep == HIGH ? LOW : HIGH;
                digitalWrite(DIR, stepper_direction);
                digitalWrite(PUL, motorStep);
            }
            xSemaphoreGive(xSemaphore);
        }
    }
}

void microRosLoop(void *pvParameters)
{
    set_microros_serial_transports(Serial);

    // Create the microROS node. Template parameters are:
    // <Number of publisher, Number of timers, Number of subscriber>.
    RoverMicroRosLib::Node<0, 1u, 2u> node(NAME_NS, NAME_NODE);
    // Necessary
    node.init();

    // Link global publisher, timer and subscriber with the node, this will
    // later allow the node to call subscriber and timer callback when those are
    // ready (on new data for example).
    node.addSubscriber(&subHeartbeat);
    node.addSubscriber(&subGoal);

    for (EVER)
    {
        // This handle the connection with the ROS network and calls ready
        // callbacks.
        node.spinSome(RCL_MS_TO_NS(0UL));
    }
}

void cbGoal(const void *msg_)
{
    unsigned long step_timer;
    const rover_msgs__msg__AntennaCmd *msg = (rover_msgs__msg__AntennaCmd *)msg_;
    // LOG(INFO, "Received: %f", msg->speed);

    if (timer_heartbeat.isDone())
    {
        root_heartbeat_state = false;
    }

    if (msg->speed != 0 && root_heartbeat_state)
    {
        step_timer = 2 * PI * 1e6 / (MICRO_STEPS * abs(msg->speed) * RATIO_MOTOR);
        // LOG(INFO, "Received: %i", step_timer);

        if (xSemaphoreTake(xSemaphore, (TickType_t)100) == pdTRUE)
        {
            if (msg->speed < 0)
            {
                stepper_direction = false;
            }
            else
            {
                stepper_direction = true;
            }

            motor_status = true;
            timer.updateInterval(step_timer);
            xSemaphoreGive(xSemaphore);
        }
    }
    else
    {
        motor_status = false;
    }
}

void cbSubHeartbeat(const void *msg_)
{
    root_heartbeat_state = true;
    timer_heartbeat.init(500);
    LOG(INFO, "heartbeat: %d", root_heartbeat_state);
}

// Do not use when using FreeRTOS.
void loop() {}
