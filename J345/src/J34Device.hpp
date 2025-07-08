#ifndef J34_DEVICE_HPP
#define J34_DEVICE_HPP

#include "J34Actuator.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

// #include "rover_can2/msgs/" // Todo: Generate msg file
// #include "rover_can2/msgs/" // Todo: Generate msg file
// #include "rover_can2/msgs/" // Todo: Generate msg file

class J34Device
{
    static constexpr uint64_t LOOP_PERIOD = 250ULL;

  public:
    J34Device() = default;

    void init()
    {
        j34.init();
    }

    void update()
    {
        if (!loopTimer.isReady())
        {
            return;
        }

        j34.update();

        if (pbCalib.isClicked())
        {
            j34.calib(0.0F, 0.0F);
        }

        if (pbFwd.isClicked())
        {
            j34.setSpeeds(-10.0F, 0.0F);
        }
        else if (pbRev.isClicked())
        {
            j34.setSpeeds(10.0F, 0.0F);
        }
        else if (pbRight.isClicked())
        {
            j34.setSpeeds(0.0F, 10.0F);
        }
        else if (pbLeft.isClicked())
        {
            j34.setSpeeds(0.0F, -10.0F);
        }
        else
        {
            j34.setSpeeds(0.0F, 0.0F);
        }
    }

  private:
    LoopTimer<uint64_t, &Time::micros> loopTimer = {LOOP_PERIOD};
    J34Actuator j34;

    PushButton pbCalib = {PIN_PB_J34_CALIB};
    PushButton pbFwd = {PIN_PB_J3_FWD};
    PushButton pbRev = {PIN_PB_J3_REV};
    PushButton pbRight = {PIN_PB_J4_FWD};
    PushButton pbLeft = {PIN_PB_J4_REV};

    VALIDATE_CONCEPT(RoverObject, J34Device);
};

#endif  // J34_DEVICE_HPP
