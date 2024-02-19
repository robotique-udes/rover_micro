#include <Arduino.h>

#include <std_msgs/msg/int32.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "helpers/log.hpp"

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
void microRosLoop(void *pvParameters);
void cbTimer(rcl_timer_t *timer, int64_t last_call_time);
void cbSubscriber(const void *msg_);

SemaphoreHandle_t MutexLoopAvgTime = xSemaphoreCreateMutex();
static float loopAvgTime = 0;

// Global objects
RoverMicroRosLib::Publisher pub = RoverMicroRosLib::Publisher(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pub_test_topic");
RoverMicroRosLib::Timer timer = RoverMicroRosLib::Timer(RCL_MS_TO_NS(100), cbTimer);
RoverMicroRosLib::Subscriber sub = RoverMicroRosLib::Subscriber(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pub_test_topic", cbSubscriber);

void setup()
{
    Serial.begin(115200);

    xTaskCreatePinnedToCore(microRosLoop, "", 4096, NULL, 1, NULL, 0);

    Timer<unsigned long, micros> timer(3u);
    Timer<unsigned long, millis> timerLog(1000u);
    uint8_t motorStep = HIGH;
    uint8_t PIN_PULSE = 10u;

    Chrono<unsigned long, micros> chrono;
    uint64_t counter;
    for (EVER)
    {
        if (timer.done())
        {
            motorStep = motorStep == HIGH ? LOW : HIGH; // Switch between low and high each loop
            digitalWrite(PIN_PULSE, motorStep);
            counter++;

            if (counter > 50'000UL)
            {
                xSemaphoreTake(MutexLoopAvgTime, portMAX_DELAY);
                loopAvgTime = (float)chrono.getTime() / 50'000.0f;
                xSemaphoreGive(MutexLoopAvgTime);
                counter = 0;
                chrono.restart();
            }
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
        xSemaphoreTake(MutexLoopAvgTime, portMAX_DELAY);
        LOG(INFO, "%f", loopAvgTime);
        xSemaphoreGive(MutexLoopAvgTime);
        node.spinSome(RCL_MS_TO_NS(0UL));
    }
}

// // This function is called each at the set timer frequency
void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
{
    std_msgs__msg__Int32 msg;
    msg.data = 88L;
    pub.publish(&msg);
}

void cbSubscriber(const void *msg_)
{
    const std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32 *)msg_;
    // LOG(INFO, "Received: %i", msg->data);
}

void loop() {}
