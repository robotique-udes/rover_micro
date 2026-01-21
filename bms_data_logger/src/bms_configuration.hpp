#ifndef BMS_CONFIGURATION_HPP
#define BMS_COFIGURATION_HPP

#include <stdio.h>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

namespace BMS_CONFIG
{
    constexpr gpio_num_t CAN_TX = GPIO_NUM_21;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_22;
    constexpr int BMS_NODE_ID = 0x01;
    constexpr __uint16_t BMS_TOTAL_CELLS = 10;

    namespace OBJ_DICT
    {
        struct canOpenEntry
        {
            constexpr uint16_t index;
            constexpr uint16_t sub;
        }

        constexpr CanOpenEntry AMPS_RO = {0x2100, 0x01};
        constexpr CanOpenEntry BATT_VOLT_RO = {0x210D, 0x01};
        constexpr CanOpenEntry LOAD_VOLT_RO = {0x210D, 0x02};
        constexpr CanOpenEntry CHG_VOLT_RO = {0x210D, 0x03};
        constexpr CanOpenEntry 
    }
}



#endif