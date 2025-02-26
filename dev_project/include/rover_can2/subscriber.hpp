#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/msgs/msg.hpp"

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/static_array.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"

DEFINE_LOG_NODE(Subscriber, Logger::eNodeState::OFF)

namespace RoverCan2
{
    class SubscriberBase
    {
      public:
        Msgs::Msg::eLoadMsgCode parseMsg(const CanMsg& msgCan_)
        {
            Msgs::Msg::eLoadMsgCode loadCode = this->getMsg().get().loadMsg(msgCan_);
            switch (loadCode)
            {
                case Msgs::Msg::eLoadMsgCode::SUCCESS_COMPLETE:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Loaded last part of message, calling related callback");
                    this->triggerCallback();
                    break;
                case Msgs::Msg::eLoadMsgCode::SUCCESS_INCOMPLETE:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Loaded a part of message, waiting on for the reset...");
                    break;
                case Msgs::Msg::eLoadMsgCode::NOT_CONCERNED:
                    LOG_DEBUG(Logger::Nodes::Subscriber, "Parsed msg wasn't meant for this subscriber");
                    break;
                case Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH:
                    LOG_ERROR(Logger::Nodes::Subscriber, "Missmatch between sender and receiver, dropping message");
                    break;
                case Msgs::Msg::eLoadMsgCode::ERROR_IMPLEMENTATION:
                    ASSERT(false, "Message implementation is eronous, expect undefined behavior");
                    break;
            }

            return loadCode;
        }

      protected:
        virtual std::reference_wrapper<Msgs::Msg> getMsg(void) = 0;
        virtual void triggerCallback(void) = 0;
    };

    template<typename MSG_TYPE>
    class SubscriberBaseT : public SubscriberBase
    {
        static_assert(std::is_base_of<Msgs::Msg, MSG_TYPE>::value,
                      "Subscriber's message type must be of base class RoverCan2::Msgs::Msg");

      public:
      protected:
        std::reference_wrapper<Msgs::Msg> getMsg(void)
        {
            return this->_msg;
        }

        void triggerCallback(void)
        {
            this->processCallback(_msg);
        }

        virtual void processCallback(const MSG_TYPE msg_) = 0;

      private:
        MSG_TYPE _msg;
    };

    /**
     * @brief Subscriber -> Links the reception of a RoverCan2::Msgs to a user
     * specified callback. SubscriberStandalone is meant for standalone
     * functions (static/C style/Non-Member) callback without any dynamic
     * allocation.
     *
     * @tparam MSG_TYPE Object will subscribe to this message type
     * @tparam CLASS_TYPE
     */
    template<typename MSG_TYPE, typename CALLBACK_TYPE>
    class SubscriberStandalone : public SubscriberBaseT<MSG_TYPE>
    {
        static_assert(std::is_invocable_r<void, CALLBACK_TYPE, const MSG_TYPE&>::value,
                      "Callback must be of type: (void)(const MSG_TYPE&)");

      public:
        explicit SubscriberStandalone(CALLBACK_TYPE callback_):
            _callback(callback_)
        {
        }

      protected:
        void processCallback(const MSG_TYPE msg_) override
        {
            this->_callback(msg_);
        }

      private:
        CALLBACK_TYPE* _callback;
    };

    /**
     * @brief Subscriber -> Links the reception of a RoverCan2::Msgs to a user
     * specified callback. SubscriberMember is meant for class member callback
     * function wihtout dynamic allocation
     *
     * @tparam MSG_TYPE Object will subscribe to this message type
     * @tparam CLASS_TYPE
     */
    template<typename MSG_TYPE, typename CLASS_TYPE>
    class SubscriberMember : public SubscriberBaseT<MSG_TYPE>
    {
        typedef void (CLASS_TYPE::*MemberCallback_t)(const MSG_TYPE&);

      public:
        explicit SubscriberMember(CLASS_TYPE* context_, MemberCallback_t callback_):
            _context(context_),
            _callback(callback_)
        {
        }

      protected:
        void processCallback(const MSG_TYPE msg_) override
        {
            (_context->*_callback)(msg_);
        }

      private:
        CLASS_TYPE* _context;
        MemberCallback_t _callback;
    };
}  // namespace RoverCan2

#endif  // SUBSCRIBER_HPP
