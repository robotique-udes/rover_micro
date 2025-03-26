#ifndef CAN_MSG_HPP
#define CAN_MSG_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/helpers.hpp"

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
            _canID = canID_;

            if (!data_ || dataLength_ > msgData.size())
            {
                dataLength = msgData.size();
                std::memset(msgData.data(), 0, msgData.size());
                _msgID = Constant::eMsgId::INVALID;
                _msgContentID = 0U;
            }
            else
            {
                dataLength = dataLength_;
                std::memcpy(msgData.data(), data_, dataLength_);
                _msgID = this->getMsgID();
                _msgContentID = this->getMsgContentID();
            }
        }

        CanMsg()
        {
            CanMsg(Constant::eDeviceId::NOT_SET, nullptr, 0U);
        }

        /**
         * @brief
         * @attention Manually setting field can lead to badly constructed msgs.
         *  Prefer using a Msg::getCanMsg() instead.
         *
         * @param canID_
         */
        void setCanID(Constant::eDeviceId canID_)
        {
            _canID = canID_;
        }

        Constant::eDeviceId getCanID(void) const
        {
            return _canID;
        }

        /**
         * @brief
         * @attention Manually setting field can lead to badly constructed msgs.
         *  Prefer using a Msg::getCanMsg() instead.
         *
         * @param msgId_
         */
        void setMsgID(RoverCan2::Constant::eMsgId msgId_)
        {
            msgData[TO_UNDERLYING(Constant::eDataIndex::MSG_ID)] = TO_UNDERLYING(msgId_);
        };

        RoverCan2::Constant::eMsgId getMsgID() const
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

        /**
         * @brief
         * @attention Manually setting field can lead to badly constructed msgs.
         *  Prefer using a Msg::getCanMsg() instead.
         *
         * @param ID_
         */
        void setMsgContentID(uint8_t ID_)
        {
            msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::MSG_CONTENT_ID)] = ID_;
        };

        uint8_t getMsgContentID() const
        {
            if (dataLength < (TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)))
            {
                return 0U;
            }

            return msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::MSG_CONTENT_ID)];
        };

        uint8_t dataLength;
        std::array<uint8_t, 8> msgData = {};

      private:
        RoverCan2::Constant::eDeviceId _canID;
        RoverCan2::Constant::eMsgId _msgID;
        uint8_t _msgContentID;
    };
}  // namespace RoverCan2

#endif  // CAN_MSG_HPP
