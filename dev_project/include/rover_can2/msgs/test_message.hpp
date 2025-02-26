#ifndef TEST_MSG_HPP
#define TEST_MSG_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

#if defined(ARDUINO_ESP32S3_DEV)
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/static_array.hpp"
#include "rover_lib2/helpers/log.hpp"
#endif  // defined(ARDUINO_ESP32S3_DEV)

DEFINE_LOG_NODE(MotorCmd, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class TestMsg : public Msg
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            eSTART,
            CMD,
            CLOSE_LOOP,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float cmd;
            bool closeLoop;
        };

        static constexpr StaticArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST) - 1U> VALID_MSG_IDS
            = {eMsgContentID::CMD, eMsgContentID::CLOSE_LOOP};

      public:
        TestMsg():
            Msg(Constant::eMsgId::TEST_MSG)
        {
            _data.cmd = static_cast<decltype(_data.cmd)>(0);
            _data.closeLoop = static_cast<decltype(_data.closeLoop)>(0);
        }

        eLoadMsgCode loadMsg(const CanMsg& msg) override
        {
            if (!(msg.msgID == this->getMsgId()))
            {
                return eLoadMsgCode::NOT_CONCERNED;
            }

            eMsgContentID msgContentId = static_cast<eMsgContentID>(msg.msgContentID);
            if (!VALID_MSG_IDS.contains(msgContentId))
            {
                LOG_DEBUG(Logger::Nodes::MotorCmd,
                          "Missmatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISSMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::CMD:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg, _data.cmd);
                    LOG_DEBUG(Logger::Nodes::MotorCmd,
                              "switch (msgContentId) case eMsgContentID::CMD: %s",
                              success ? "success" : "failed");
                    break;
                case eMsgContentID::CLOSE_LOOP:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg, _data.closeLoop);
                    LOG_DEBUG(Logger::Nodes::MotorCmd,
                              "switch (msgContentId) case case eMsgContentID::CLOSE_LOOP: %s",
                              success ? "success" : "failed");
                    break;
                default:
                    return eLoadMsgCode::ERROR_IMPLEMENTATION;
            }

            if (!success)
            {
                return eLoadMsgCode::ERROR_MISSMATCH;
            }

            if (Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg))
            {
                return eLoadMsgCode::SUCCESS_COMPLETE;
            }
            else
            {
                return eLoadMsgCode::SUCCESS_INCOMPLETE;
            }
        }

        void sendMsg(const Constant::eDeviceId& senderID_) override
        {
                // CanMsg msg(senderID_, )
        }

        const sMsgData& data(void) const
        {
            return _data;
        }

      private:
        sMsgData _data;
    };
}  // namespace RoverCan2::Msgs

#endif  // TEST_MSG_HPP
