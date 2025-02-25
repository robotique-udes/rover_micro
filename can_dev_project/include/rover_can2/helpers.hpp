#ifndef HELPERS_HPP
#define HELPERS_HPP

#if defined(ARDUINO_ESP32S3_DEV)
#include <cstdint>
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"
#endif  // defined(ARDUINO_ESP32S3_DEV)

#include "rover_can2/constant.hpp"
#include "rover_can2/can_msg.hpp"

#include <optional>

DEFINE_LOG_NODE(Can_Helpers, Logger::eNodeState::OFF)

namespace RoverCan2::Helpers
{
    template<typename ROVER_MSG_CONTENT_TYPE>
    constexpr bool DATA_LENGTH_MATCHES_MSG_CONTENT(uint8_t dataLength_)
    {
        LOG_DEBUG(Logger::Nodes::Can_Helpers,
                  "DATA_LENGTH_MATCHES_MSG_CONTENT(dataLength_ = %u): dataLength_ == (sizeof(ROVER_MSG_CONTENT_TYPE) - "
                  "TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA)) -> %u == %u - %u ?)",
                  dataLength_,
                  dataLength_,
                  sizeof(ROVER_MSG_CONTENT_TYPE),
                  TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA));
        return dataLength_ == (sizeof(ROVER_MSG_CONTENT_TYPE) + TO_UNDERLYING(Constant::eDataIndex::START_OF_DATA));
    }

    template<typename ROVER_MSG_CONTENT_TYPE>
    constexpr bool CAN_MSG_TO_ROVER_MSG_CONTENT(const CanMsg& canMsg_, ROVER_MSG_CONTENT_TYPE& msgContent_)
    {
        static_assert(sizeof(ROVER_MSG_CONTENT_TYPE) <= (RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                         - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)));

        if (!DATA_LENGTH_MATCHES_MSG_CONTENT<ROVER_MSG_CONTENT_TYPE>(canMsg_.dataLength))
        {
            LOG_DEBUG(Logger::Nodes::Can_Helpers, "DATA_LENGTH_MATCHES_MSG_CONTENT Failed");
            return false;
        }
        else
        {
            msgContent_ = static_cast<ROVER_MSG_CONTENT_TYPE>(
                canMsg_.msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)]);
            return true;
        }
    }

    template<typename ROVER_MSG_ENUM_TYPE>
    constexpr bool MSG_CONTENT_IS_LAST_ELEM(const CanMsg& canMsg_)
    {
        return canMsg_.msgData[(size_t)RoverCan2::Constant::eDataIndex::MSG_CONTENT_ID]
               == ((size_t)ROVER_MSG_ENUM_TYPE::eLAST - 1);
    }
}  // namespace RoverCan2::Helpers

#endif  // HELPERS_HPP
