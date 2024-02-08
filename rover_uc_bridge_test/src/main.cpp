#include <Arduino.h>

#include "rover_ros_serial/rover_ros_serial.hpp"
#include "rover_msgs/msg/Gps.hpp"

void computeGpsMsg();

void setup()
{
    Serial.begin(1'500'000, SERIAL_8N1);

    LOG(eLoggerLevel::WARN, "Init...");
    RoverRosSerial::Node<0u> node;

    char buffer[200];
    uint32_t counter = 0u;
    Chrono<unsigned long, micros> timerLoop;
    for (EVER)
    {
        counter++;
        computeGpsMsg();
        node.spinSome();

        // sprintf(buffer, "%u", counter);
        // LOG(INFO, buffer);
        if (counter >= 100'000u)
        {
            break;
        }
    }

    sprintf(buffer, "Loop time is approx: %f", (static_cast<float>(timerLoop.getTime()/(100'000.0f*1'000'000.0f))));
    LOG(INFO, buffer);
}

void computeGpsMsg()
{
    RoverRosSerial::rover_msgs::msg::Gps gpsMsg;
    gpsMsg.uMsg.packetMsg.heading = 10.0f;

    gpsMsg.sendMsg();
}
