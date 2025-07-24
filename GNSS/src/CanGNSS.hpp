#include "GNSSManager.hpp"
#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

constexpr uint32_t PUBLISH_PERIOD_FAST_MS = 50UL;
constexpr uint32_t PUBLISH_PERIOD_SLOW_MS = 1000UL;

class CanGNSS : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>,
                                         RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>
{
  public:
    CanGNSS():
        RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>,
                          RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>,
                          RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>>(RoverCan2::Constant::eDeviceId::GNSS,
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixPosition>(),
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixHeading>(),
                                                                          RoverCan2::Publisher<RoverCan2::Msgs::FixInfo>()),
        updateTimerFast(PUBLISH_PERIOD_FAST_MS),
        updateTimerSlow(PUBLISH_PERIOD_SLOW_MS)
    {
    }

    void _init(void) {}

    void _update(void)
    {
        if (updateTimerFast.isReady())
        {
            this->sendMsg(posMsg);
            this->sendMsg(headingMsg);
        }
        if (updateTimerSlow.isReady())
        {
            this->sendMsg(infoMsg);
        }
    }

    void set(const sGNSSData& newData_)
    {
        posMsg.data().latitude = newData_.latitude;
        posMsg.data().longitude = newData_.longitude;

        headingMsg.data().headingDeg = newData_.headingDeg;

        infoMsg.data().fixQuality = newData_.fixQuality;
        infoMsg.data().headingQuality = newData_.headingQuality;
        infoMsg.data().satelliteCount = newData_.satellites;
    }

  private:
    RoverCan2::Msgs::FixPosition posMsg;
    RoverCan2::Msgs::FixHeading headingMsg;
    RoverCan2::Msgs::FixInfo infoMsg;
    LoopTimer<uint64_t, Time::millis> updateTimerFast;
    LoopTimer<uint64_t, Time::millis> updateTimerSlow;
};