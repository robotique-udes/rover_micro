#include <stdio.h>
#include "driver/twai.h"

#define CAN_TX_GPIO GPIO_NUM_21
#define CAN_RX_GPIO GPIO_NUM_22
#define bmsNodeID 0x01

void send_query(uint16_t index, uint8_t sub_index)
{
    twai_message_t msg;
    msg.identifier = 0x600 + bmsNodeID;
    msg.data_length_code = 8;
    msg.flags = TWAI_MSG_FLAG_NONE;

    msg.data[0] = (index >> 8) & 0xFF;
    msg.data[1] = index & 0xFF;
    msg.data[2] = sub_index;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    twai_transmit(&msg, pdMS_TO_TICKS(1000));
}

extern "C" void app_main(void)
{

    printf("BMS CAN logger starting...\n");

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}