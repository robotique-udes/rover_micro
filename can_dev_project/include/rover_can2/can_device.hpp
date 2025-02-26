#ifndef CAN_DEVICE_HPP
#define CAN_DEVICE_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/msgs/msg.hpp"

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/static_array.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"

DEFINE_LOG_NODE(CanDevice, Logger::eNodeState::OFF)

namespace RoverCan2
{
    template<typename MSG_TYPE, typename CALLBACK_TYPE>
    class Subscriber
    {
        static_assert(std::is_base_of<Msgs::Msg, MSG_TYPE>::value,
                      "Subscriber's message type must be of base class RoverCan2::Msgs::Msg");
        static_assert(std::is_trivially_copyable_v<CALLBACK_TYPE>,
                      "Can't provide heap allocated func ptr, use lambda type instead");
        static_assert(std::is_invocable_r<void, CALLBACK_TYPE, const MSG_TYPE&>::value,
                      "Callback must be of type: (void)(const MSG_TYPE&)");

      public:
        explicit Subscriber(CALLBACK_TYPE callback_):
            _callback(callback_)
        {
        }

        void parseMsg(const CanMsg& msgCan_)
        {
            Msgs::Msg::eLoadMsgCode errorCode = _msg.loadMsg(msgCan_);
            switch (errorCode)
            {
                case Msgs::Msg::eLoadMsgCode::SUCCESS_COMPLETE:
                    LOG_DEBUG(Logger::Nodes::CanDevice, "Loaded last part of message, calling related callback");
                    this->_callback(_msg);
                    break;
                case Msgs::Msg::eLoadMsgCode::SUCCESS_INCOMPLETE:
                    LOG_DEBUG(Logger::Nodes::CanDevice, "Loaded a part of message, waiting on for the reset...");
                    break;
                case Msgs::Msg::eLoadMsgCode::NOT_CONCERNED:
                    LOG_DEBUG(Logger::Nodes::CanDevice, "Parsed msg wasn't meant for this subscriber");
                    break;
                case Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH:
                    LOG_ERROR(Logger::Nodes::CanDevice, "Missmatch between sender and receiver, dropping message");
                    break;
                case Msgs::Msg::eLoadMsgCode::ERROR_IMPLEMENTATION:
                    ASSERT(false, "Message implementation is eronous, expect undefined behavior");
                    break;
            }
        }

      private:
        MSG_TYPE _msg;
        CALLBACK_TYPE _callback;
    };

    // Factory for deduction
    template<typename MSG_TYPE, typename CALLBACK_TYPE>
    Subscriber<MSG_TYPE, CALLBACK_TYPE> createSubscription(CALLBACK_TYPE&& callback)
    {
        return Subscriber<MSG_TYPE, CALLBACK_TYPE>(std::forward<CALLBACK_TYPE>(callback));
    }

    class CanDevice : public RoverObject
    {
      public:
        CanDevice(RoverCan2::Constant::eDeviceId id_):
            _id(id_)
        {
        }

      private:
        RoverCan2::Constant::eDeviceId _id;
        // StaticArray<RoverCan2::Constant::eMsgId> _subscribedMsgId;
    };
}  // namespace RoverCan2

#endif  // CAN_DEVICE_HPP
