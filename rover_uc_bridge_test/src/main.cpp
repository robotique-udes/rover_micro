#include <Arduino.h>
#include "helpers/macros.h"
#include <arpa/inet.h>

struct rover_msgs__msg__AntennaCmd;
union rover_msgs__msg__AntennaCmd__packet;

void buildAndSendData(void *pvParameters);
rover_msgs__msg__AntennaCmd__packet msgToPacket(rover_msgs__msg__AntennaCmd &msg);

typedef struct rover_msgs__msg__AntennaCmd
{
    bool status;
    float speed;
} rover_msgs__msg__AntennaCmd;

union rover_msgs__msg__AntennaCmd__packet
{
    rover_msgs__msg__AntennaCmd msg;
    uint8_t data[sizeof(rover_msgs__msg__AntennaCmd)];
};

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // LOG(INFO, "Starting program");

    // Start buildAndSendData on the second core
    xTaskCreatePinnedToCore(buildAndSendData, NULL, 4096, NULL, 1, NULL, 1);
}

void buildAndSendData(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (EVER)
    {
        // LOG(INFO, "Building msg...\r");
        // Serial.write((int8_t)-10);

        rover_msgs__msg__AntennaCmd msg;
        msg.speed = 25000.0f;
        msg.status = true;

        rover_msgs__msg__AntennaCmd__packet packet = msgToPacket(msg);

        Serial.write(packet.data, sizeof(packet.data));

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000u));
    }
}

rover_msgs__msg__AntennaCmd__packet msgToPacket(rover_msgs__msg__AntennaCmd &msg)
{
    rover_msgs__msg__AntennaCmd__packet packet;
    packet.msg = msg;

    return packet;
}
