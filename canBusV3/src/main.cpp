#include <Arduino.h>
#include "driver/can.h"
#include "can_lib.h"

class can_lib can_lib;

void setup()
{
    can_lib.can_init(GPIO_NUM_4, GPIO_NUM_18);
}

//==================================================================================//

void loop()
{
    uint8_t *datas;
    delay(1000);
    uint8_t data[2];
    data[0] = 0xFA;
    data[1] = 0xDE;

    can_lib.can_write_msg(12u, 2, data);

    can_lib.can_lib_update();
    // if (can_lib.isNewMsg())
    // {
    //     printf("Message received\n");
    //     datas = can_lib.can_get_data();
    //     int id = can_lib.can_get_id();
    //     if (id == 0x123)
    //     {
    //         can_lib.can_write_msg(0x069, 2, data);
    //         printf("msg sent");
    //     }
    //     printf("ID is %d\n", can_lib.can_get_id());
    //     for (int i = 0; i < can_lib.can_get_lenght(); i++)
    //     {
    //         printf("Data byte %d = %d\n", i, datas[i]);
    //     }
    // }
}
