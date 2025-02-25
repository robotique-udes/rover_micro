#ifndef MSG_HPP
#define MSG_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/can_msg.hpp"

namespace RoverCan2::Msgs
{
    class Msg
    {
      public:
        enum class eLoadMsgCode
        {
            SUCCESS_COMPLETE,
            SUCCESS_INCOMPLETE,
            NOT_CONCERNED,
            ERROR_MISSMATCH,
            ERROR_IMPLEMENTATION
        };

        Msg(RoverCan2::Constant::eMsgId msgId_):
            _msgID(msgId_)
        {
        }

        constexpr RoverCan2::Constant::eMsgId getMsgId() const
        {
            return _msgID;
        }

        /**
         * @brief
         *
         * @param msg
         * @return eLoadMsgCode
         *    SUCCESS_COMPLETE: Successfully loaded the last element of a msg
         *    SUCCESS_INCOMPLETE: Successfully loaded any element of a msg other than the last
         *    NOT_CONCERNED: Msg ID doesn't match
         *    ERROR_MISSMATCH: Missmatching message definition between the one received and the local on the device
         *    ERROR_IMPLEMENTATION: Shouldn't return this error code, it mean the message definition itself is erronous
         */
        virtual eLoadMsgCode loadMsg(const CanMsg& msg) = 0;

      private:
        const RoverCan2::Constant::eMsgId _msgID;
    };
}  // namespace RoverCan2::Msgs

#endif  // MSG_HPP
