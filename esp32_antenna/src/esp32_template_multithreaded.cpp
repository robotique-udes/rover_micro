#include <Arduino.h>

#if !defined (ESP32)
#error CPU not supported
#endif

#include <std_msgs/msg/int32.h>
#include "rover_msgs/msg/antenna_cmd.h"

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "helpers/helpers.hpp"

#include <semaphore>

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

#define PUL 25
#define DIR 26
#define EN 27

#define PI 3.1415
#define MICRO_STEPS 800
#define RATIO_MOTOR 77
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
void cbSubscriber(const void *msg_);

// Global objects
RoverMicroRosLib::Subscriber sub = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, AntennaCmd), "/base/antenna/cmd/out/goal", cbSubscriber);
Timer<unsigned long, micros> timer(200ul);
bool motor_status = false;
bool stepper_direction;

// Creating mutex
SemaphoreHandle_t xSemaphore = NULL;

void setup()
{
    Serial.begin(115200);
    pinMode(PUL, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(DIR,OUTPUT);

    xTaskCreatePinnedToCore(microRosLoop, "", 8192, NULL, 1, NULL, 0);

    xSemaphore = xSemaphoreCreateMutex();

    uint8_t motorStep = HIGH;
    uint8_t PIN_PULSE = 10u;

    digitalWrite(EN, LOW);
    digitalWrite(DIR, HIGH);


    for (EVER)
    {
        if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
        {
            if (timer.isDone() && motor_status)
            {
                // Switch between LOW and HIGH each loop at period/2
                motorStep = motorStep == HIGH ? LOW : HIGH;
                // digitalWrite(PIN_PULSE, motorStep);
                digitalWrite(DIR, stepper_direction);
                digitalWrite(PUL, motorStep);
            }
            xSemaphoreGive( xSemaphore );
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
    RoverMicroRosLib::Node<0, 0, 1u> node(NAME_NS, NAME_NODE);
    // Necessary
    node.init();

    // Link global publisher, timer and subscriber with the node, this will
    // later allow the node to call subscriber and timer callback when those are
    // ready (on new data for example).
    node.addSubscriber(&sub);

    for (EVER)
    {
        // This handle the connection with the ROS network and calls ready 
        // callbacks.
        node.spinSome(RCL_MS_TO_NS(0UL));
    }
}


// This function is called each time new data is available on the 
// /pub_test_topic topic.
void cbSubscriber(const void *msg_)
{
    // Cast the void pointer into the correct message type for the topic. This 
    // is necessary to access the data in the correct format. It then logs the
    // msg data into the terminal logger. 
    // (ros2 launch rover_helper terminal_logger.launch.py)

    unsigned long step_timer;
    const rover_msgs__msg__AntennaCmd *msg = (rover_msgs__msg__AntennaCmd *)msg_;
    //LOG(INFO, "Received: %f", msg->speed);
    if(msg->speed != 0)
    {
        step_timer = 2 * PI * 1e6 / (MICRO_STEPS * abs(msg->speed) * RATIO_MOTOR);
        LOG(INFO, "Received: %i", step_timer);

        if(xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE)
        {
            if(msg->speed < 0)
            {
                stepper_direction = false;
            }
            else
            {
                stepper_direction = true;
            }

            motor_status = true;
            timer.updateInterval(step_timer);
            xSemaphoreGive( xSemaphore );
        }
    }
    else
    {
        motor_status = false;
    }
    //LOG(INFO, "Received: %i", step_timer);
}

// Do not use when using FreeRTOS.
void loop() {}
