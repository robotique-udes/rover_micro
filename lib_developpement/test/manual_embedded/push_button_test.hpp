#include <Arduino.h>
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/sensors/push_button.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup()
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    PushButton pb_J3_R(GPIO_NUM_41);
    PushButton pb_J3_F(GPIO_NUM_42);
    PushButton pb_J4_R(GPIO_NUM_38);
    PushButton pb_J4_F(GPIO_NUM_2);
    PushButton pb_J5_O(GPIO_NUM_1);
    PushButton pb_J5_C(GPIO_NUM_39);
    PushButton pb_CALIB(GPIO_NUM_40);

    for (EVER)
    {
        if (pb_J3_R.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J3_R");
        }
        if (pb_J3_F.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J3_F");
        }
        if (pb_J4_R.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J4_R");
        }
        if (pb_J4_F.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J4_F");
        }
        if (pb_J5_O.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J5_O");
        }
        if (pb_J5_C.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_J5_C");
        }
        if (pb_CALIB.isClicked())
        {
            LOG_INFO(Logger::Nodes::Main, "pb_CALIB");
        }
    }
}

void loop() {}
