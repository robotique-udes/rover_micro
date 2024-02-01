#include <Arduino.h>
#include <arpa/inet.h>

#define ROVER_ROS_SERIAL

#include "helpers/macros.h"
#include "rover_ros_serial.hpp"
#include "rover_msgs/msg/rover_msgs__msg__antenna_cmd.hpp"

void buildAndSendData(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    delay(500);
    // while (!Serial)
    //     ;

    // LOG(INFO, "Starting program");

    // Start buildAndSendData on the second core
    xTaskCreatePinnedToCore(buildAndSendData, NULL, 4096, NULL, 1, NULL, 0);
}

void buildAndSendData(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    RoverRosSerial::MsgLogger logMsg;
    for (EVER)
    {
        logMsg.setLog("This is a first msg!");
        logMsg.sendMsg(&Serial);

        logMsg.setLog("This is a second msg!");
        logMsg.sendMsg(&Serial);

        // xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000u));
    }
}
