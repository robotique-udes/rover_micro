#include "can_lib.h"
void can_lib::can_init(gpio_num_t tx, gpio_num_t rx){
      //Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(tx, rx, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    //Install CAN driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start CAN driver
    if (can_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }

}

uint8_t can_lib::can_write_msg(uint32_t id, uint8_t data_lenght,uint8_t* data){
//Configure message to transmit
can_message_t message;
message.identifier = id;
message.flags = CAN_MSG_FLAG_EXTD;
message.data_length_code = data_lenght;
for (int i = 0; i < data_lenght; i++) {
    message.data[i] = data[i];
}

//Queue message for transmission
if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    return 0;//Message queued for transmission\n
} else {
    return 1;//Failed to queue message for transmission
}
}