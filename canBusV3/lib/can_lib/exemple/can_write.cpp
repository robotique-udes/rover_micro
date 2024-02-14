#include <Arduino.h>
#include "driver/can.h"
#include "can_lib.h"
class can_lib can_lib;

void setup() {
  can_lib.can_init(GPIO_NUM_4, GPIO_NUM_5);
}
//hello
//==================================================================================//

void loop() {
  uint8_t data[2];
  data[0] = 0xBE;
  data[1] = 0xEF;
  can_lib.can_write_msg(0x069, 2, data);
  delay(1000);
}
