#include <Arduino.h>

#include "rover_ros_serial/rover_ros_serial.hpp"

void mainLoop(void *pvParameters);

void setup()
{
    Serial.begin(115200, SERIAL_8N1);

    // Start taskCommunication on the second core
    xTaskCreatePinnedToCore(mainLoop, NULL, 8'192, NULL, 1, NULL, 0);
}

void mainLoop(void *pvParameters)
{
    LOG(eLoggerLevel::WARN, "Init...");
    Heartbeat hearbeat(10u, &Serial);

    for (EVER)
    {
        hearbeat.update();
    }
}
