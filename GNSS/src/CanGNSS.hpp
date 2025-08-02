#include "GNSSManager.hpp"
#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

constexpr uint32_t PUBLISH_PERIOD_TELEMETRY_MS = 50UL;
constexpr uint32_t PUBLISH_PERIOD_MSG_QUALITY_MS = 1000UL;

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
        _updateTimerTelemetry(PUBLISH_PERIOD_TELEMETRY_MS),
        _updateTimerMsgQuality(PUBLISH_PERIOD_MSG_QUALITY_MS)
    {
    }

    void _init(void) {}

    void _update(void)
    {
        if (_updateTimerTelemetry.isReady())
        {
            this->sendMsg(_posMsg);
            this->sendMsg(_headingMsg);
        }
        if (_updateTimerMsgQuality.isReady())
        {
            this->sendMsg(_infoMsg);
        }
    }

    void set(const sGNSSData& newData_)
    {
        _posMsg.data().latitude = newData_.latitude;
        _posMsg.data().longitude = newData_.longitude;

        _headingMsg.data().headingDeg = newData_.headingDeg;

        _infoMsg.data().fixQuality = newData_.fixQuality;
        _infoMsg.data().headingQuality = newData_.headingQuality;
        _infoMsg.data().satelliteCount = newData_.satellites;
    }

  private:
    RoverCan2::Msgs::FixPosition _posMsg;
    RoverCan2::Msgs::FixHeading _headingMsg;
    RoverCan2::Msgs::FixInfo _infoMsg;
    LoopTimer<uint64_t, Time::millis> _updateTimerTelemetry;
    LoopTimer<uint64_t, Time::millis> _updateTimerMsgQuality;
};