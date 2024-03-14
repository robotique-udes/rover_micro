#ifndef __HEARTBEAT_HPP__
#define __HEARTBEAT_HPP__

#include <cstdint>
#include "driver/twai.h"
#include "rover_can_lib/msgs/msg.hpp"

namespace RoverCanLib
{
    namespace Helpers
    {
        twai_message_t getErrorIdMsg(void);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void canMsgToStruct(IN const twai_message_t *msg_, OUT COPY_TYPE *dest_);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void structToCanMsg(IN const COPY_TYPE *structMember_, OUT twai_message_t *msg_);
    }
}

#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Msgs
{
    class Heartbeat : public Msg
    {
    public:
        enum class eMsgID : uint8_t
        {
            NOT_USED = 0x00,
            eLAST
        };

        struct sMsgData
        {
            uint8_t dontUse;
        };

        Heartbeat() {}
        ~Heartbeat() {}

        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {
            LOG(ERROR, "Should parse heartbeat msgs");
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t *msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::HEARTBEAT;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            return Constant::eInternalErrorCode::OK;
        }

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}

#endif // __HEARTBEAT_HPP__
