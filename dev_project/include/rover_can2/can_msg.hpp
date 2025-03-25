#ifndef CAN_MSG_HPP
#define CAN_MSG_HPP

#include "rover_can2/constant.hpp"

#if defined(ARDUINO_ESP32S3_DEV)
#include "driver/twai.h"
#endif  // defined(ARDUINO_ESP32S3_DEV)

#include <cstdint>
#include <cstring>
#include <type_traits>
#include "rover_lib2/helpers/macros.hpp"

namespace RoverCan2
{
    struct CanMsg
    {
#if defined(ARDUINO_ESP32S3_DEV)
        CanMsg(twai_message_t& twaiMsg_):
            CanMsg(static_cast<RoverCan2::Constant::eDeviceId>(twaiMsg_.identifier), twaiMsg_.data, twaiMsg_.data_length_code)
        {
        }
#endif  // defined(ARDUINO_ESP32S3_DEV)

        CanMsg(RoverCan2::Constant::eDeviceId canID_, const uint8_t* data_, uint8_t dataLength_)
        {
            canID = canID_;

            if (dataLength_ > msgData.size())
            {
                dataLength = msgData.size();
                std::memset(msgData.data(), 0, msgData.size());
                msgID = Constant::eMsgId::INVALID;
                msgContentID = 0U;
            }
            else
            {
                dataLength = dataLength_;
                std::memcpy(msgData.data(), data_, dataLength_);
                msgID = this->getMsgID();
                msgContentID = this->getMsgContentID();
            }
        }

        RoverCan2::Constant::eDeviceId canID;
        uint8_t dataLength;
        std::array<uint8_t, 8> msgData = {};
        RoverCan2::Constant::eMsgId msgID;
        uint8_t msgContentID;

      private:
        RoverCan2::Constant::eMsgId getMsgID()
        {
            if (dataLength < (TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)))
            {
                return RoverCan2::Constant::eMsgId::INVALID;
            }

            uint8_t msgIDint = msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::MSG_ID)];
            RoverCan2::Constant::eMsgId msgID = static_cast<RoverCan2::Constant::eMsgId>(msgIDint);

            if (RoverCan2::Constant::SUPPORTED_MSGS.contains(msgID))
            {
                return msgID;
            }
            else
            {
                return RoverCan2::Constant::eMsgId::INVALID;
            }
        };

        uint8_t getMsgContentID()
        {
            if (dataLength < (TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)))
            {
                return 0U;
            }

            return msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::MSG_CONTENT_ID)];
        };
    };
}  // namespace RoverCan2

#endif  // CAN_MSG_HPP
