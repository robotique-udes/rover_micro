#ifndef bn880_h
#define bn880_h
#include <Arduino.h>
#include <stdio.h>
#include "rover_msgs/msg/gps.h"
class bn880
{
private:
  HardwareSerial* serialbn880;
  rover_msgs__msg__Gps data;
  rover_msgs__msg__Gps msg;

public:
    bn880(HardwareSerial& serial);
    String string;
    ~bn880();
    void readData();
    void start(uint16_t baud = 9600, uint8_t RX_pin = 16,uint8_t TX_pin = 17);
    rover_msgs__msg__Gps getMsg();
};



#endif