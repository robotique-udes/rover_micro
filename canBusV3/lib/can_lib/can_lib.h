//Write a message with can
//Source:   https://docs.espressif.com/projects/esp-idf/en/release-v3.3/api-reference/peripherals/can.html
#ifndef CAN_LIB_H
#define CAN_LIB_H
#include <Arduino.h>
#include "driver/can.h"

class can_lib{
    private:
    public:
    uint8_t newMsg = 0;
    can_message_t message;
    can_lib(){}
    ~can_lib(){}
    void can_init(gpio_num_t tx = GPIO_NUM_4,gpio_num_t rx = GPIO_NUM_5);
    void can_start_listening();
    void can_lib_update();
    uint8_t can_write_msg(uint32_t id, uint8_t data_lenght,uint8_t* data);
    uint8_t* can_get_data();
    uint32_t can_get_id();
    uint8_t can_get_lenght();
    uint8_t isNewMsg();
};
#endif