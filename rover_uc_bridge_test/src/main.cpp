#include <Arduino.h>

#include "rover_ros_serial/rover_ros_serial.hpp"
#include "rover_msgs/msg/Gps.hpp"

void computeGpsMsg();

void setup()
{
    Serial.begin(115200, SERIAL_8N1);

    LOG(eLoggerLevel::WARN, "Init...");
    RoverRosSerial::Node<0u> node;

    for (EVER)
    {
        computeGpsMsg();
        node.spinSome();
    }
}

void computeGpsMsg()
{
    RoverRosSerial::rover_msgs::msg::Gps gpsMsg;
    gpsMsg.uMsg.packetMsg.heading = 10.0f;

    gpsMsg.sendMsg();
}
