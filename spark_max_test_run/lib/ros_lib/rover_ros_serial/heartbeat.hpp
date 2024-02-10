#ifndef __HEARTBEAT_HPP__
#define __HEARTBEAT_HPP__

#include "rover_ros_serial.hpp"

namespace RoverRosSerial
{
    class Heartbeat : protected RoverRosSerial::Msg
    {
    public:
        Heartbeat(uint8_t frequency_ = 10u, HardwareSerial *serial_ = &Serial) : Msg(serial_)
        {
            _frequency = frequency_;
            _timerHeartbeatSender.init(static_cast<unsigned long>(round(1'000.0f / static_cast<float>(frequency_))));

            uHeader.header.type = RoverRosSerial::Constant::eHeaderType::heartbeat;
            uHeader.header.length = 0u; // Sending Empty msgs
        }
        ~Heartbeat() {}

        void update()
        {
            if (_timerHeartbeatSender.isDone())
            {
                // LOG(INFO, "Sending heartbeat signal");
                this->sendMsg();
            }
        }

    private:
        uint8_t _frequency;
        Timer<unsigned long, millis> _timerHeartbeatSender;

        uint8_t *getSerializedData(void) { return NULL; };
        uint8_t getSerializedDataSize(void) { return 0; };
    };
}
#endif
