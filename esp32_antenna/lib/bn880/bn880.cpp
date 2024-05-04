#include "bn880.h"
#include "rover_helpers/helpers.hpp"
bn880::bn880(HardwareSerial& serial){
  bn880::serialbn880 = &serial;
}
bn880::~bn880(){
}
void bn880::start(uint16_t baud, uint8_t RX_pin,uint8_t TX_pin){
  bn880::serialbn880->begin(baud,SERIAL_8N1, RX_pin, TX_pin);
}
void bn880::readData(){
  if((bn880::serialbn880)->available() > 0){
      string = (bn880::serialbn880)->readStringUntil('\n');
    if (string.startsWith("$GNGGA")) {
        String tokens[15];
        int i = 0;
        int startIndex = 0;
        int endIndex = string.indexOf(',');

        while (endIndex != -1) {
          tokens[i++] = string.substring(startIndex, endIndex);
          startIndex = endIndex + 1;
          endIndex = string.indexOf(',', startIndex);
        }

        bn880::data.latitude = tokens[2].toFloat();
        bn880::data.longitude = tokens[4].toFloat();
        bn880::data.height = tokens[9].toFloat();
        bn880::data.satellite = tokens[7].toInt();
    }
  }
}

rover_msgs__msg__Gps bn880::getMsg(){
  return data;
}