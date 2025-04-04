#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <optional>

namespace RoverCan2
{
    class CanManagerT;

    class PublisherBaseT
    {
      protected:
        PublisherBaseT() = default;
    };

    template<typename MsgT, size_t MSG_QUEUE_SIZE = 10UL>
    class Publisher : public PublisherBaseT
    {
        VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);

        static constexpr size_t MAX_MSG_TRANSMISSION_PER_CALL = MSG_QUEUE_SIZE;

      public:
        Publisher() {}

        /**
         * @brief Queue a msg for transmission
         *
         * @attention [WARNING] Msgs won't be sent until a Manager "update/spin" is called
         * @param msg_
         */
        CircularBufferT::eErrorCode queueMsg(const MsgT& msg_)
        {
            return _msgToSendBuffer.addValue(msg_);
        }

        /**
         * @brief Called by the manager in its update loop, can be used directly by user
         *
         * @tparam ManagerT
         * @param senderId_
         * @param manager_
         * @return true
         * @return false
         */
        template<typename ManagerT>
        bool sendQueuedMsgs(Constant::eDeviceId senderId_, ManagerT& manager_)
        {
            VALIDATE_BASE_TYPE(CanManagerT, ManagerT);

            bool allSuccess = true;
            for (size_t i = 0; i < MAX_MSG_TRANSMISSION_PER_CALL; i++)
            {
                if (std::optional<MsgT> msgToSend = _msgToSendBuffer.getValue())
                {
                    allSuccess &= manager_.sendMsg(senderId_, msgToSend.value());
                }
            }

            return allSuccess;
        }

      private:
        CircularBuffer<MsgT, MSG_QUEUE_SIZE> _msgToSendBuffer;
    };
}  // namespace RoverCan2
#endif  // PUBLISHER_HPP
