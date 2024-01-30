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

    rover_msgs__msg__AntennaCmd antennaMsg;
    rover_ros_serial__msg__Logger logMsg;
    for (EVER)
    {
        // logMsg.msg.header = (uint8_t)rover_ros_serial::eHeaderCode::logs;
        // logMsg.msg.severity = INFO;
        // sprintf(logMsg.msg.msg, "This is a test!\r\n");
        // Serial.write(logMsg.getSerializedData(), logMsg.getSerializedDataSize());

        antennaMsg.msg.header = rover_ros_serial::eHeaderCode::publisher;
        antennaMsg.msg.speed = 20000.0f;
        antennaMsg.msg.status = true;
        antennaMsg.msg.ofl = '\n';
        Serial.write(antennaMsg.getSerializedData(), antennaMsg.getSerializedDataSize());

        antennaMsg.msg.header = rover_ros_serial::eHeaderCode::publisher + 1u;
        antennaMsg.msg.speed = 20000.0f;
        antennaMsg.msg.status = true;
        antennaMsg.msg.ofl = '\n';
        Serial.write(antennaMsg.getSerializedData(), antennaMsg.getSerializedDataSize());

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000u));
    }
}
