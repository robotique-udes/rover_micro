#ifndef MORSE_INPUT_HPP
#define MORSE_INPUT_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"


DEFINE_LOG_NODE(MorseInput_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class MorseInput : public Msg<MorseInput>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            SPEED_WPM,
            SYMBOL,
            eLAST,
        };

      private:
        struct sMsgData
        {
            uint8_t speed_wpm;
            uint8_t symbol;

            static_assert(sizeof(speed_wpm) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(symbol) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::SPEED_WPM, eMsgContentID::SYMBOL};

      public:
        MorseInput():
            Msg(Constant::eMsgId::MORSE_INPUT)
        {
            _data.speed_wpm = static_cast<decltype(_data.speed_wpm)>(0);
            _data.symbol = static_cast<decltype(_data.symbol)>(0);
        }

        eLoadMsgCode _loadMsg(const CanMsg& msg_)
        {
            if (msg_.getMsgID() == Constant::eMsgId::INVALID)
            {
                return eLoadMsgCode::ERROR_INVALID_MSG;
            }

            if (msg_.getMsgID() != this->getMsgId())
            {
                return eLoadMsgCode::NOT_CONCERNED;
            }

            eMsgContentID msgContentId = static_cast<eMsgContentID>(msg_.getMsgContentID());
            if (!VALID_MSG_IDS.contains(msgContentId))
            {
                LOG_DEBUG(Logger::Nodes::MorseInput_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::SPEED_WPM:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.speed_wpm);
                    LOG_DEBUG(Logger::Nodes::MorseInput_msg,
                              "switch (msgContentId) case eMsgContentID::SPEED_WPM: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::SYMBOL:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.symbol);
                    LOG_DEBUG(Logger::Nodes::MorseInput_msg,
                              "switch (msgContentId) case eMsgContentID::SYMBOL: %s",
                              success ? "success" : "failed");
                    break;
                case eMsgContentID::eLAST:
                    [[fallthrough]];
                default:
                    return eLoadMsgCode::ERROR_IMPLEMENTATION;
            }

            if (!success)
            {
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            if (Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_))
            {
                return eLoadMsgCode::SUCCESS_COMPLETE;
            }
            else
            {
                return eLoadMsgCode::SUCCESS_INCOMPLETE;
            }
        }

        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const
        {
            eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

            if (!VALID_MSG_IDS.contains(msgContentID))
            {
                return std::nullopt;
            }

            CanMsg msg_;
            switch (static_cast<eMsgContentID>(msgContentId_))
            {
                case eMsgContentID::SPEED_WPM:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.speed_wpm, msg_);
                    break;

                case eMsgContentID::SYMBOL:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.symbol, msg_);
                    break;
                case eMsgContentID::eLAST:
                    [[fallthrough]];
                default:
                    return std::nullopt;
            }

            return msg_;
        }

        uint8_t _getMsgContentCount(void) const
        {
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }

        sMsgData& data(void)
        {
            return _data;
        }

        const sMsgData& getData(void) const
        {
            return static_cast<const sMsgData&>(_data);
        }

      private:
        sMsgData _data;
    };

}  // namespace RoverCan2::Msgs

#endif  // MORSE_INPUT_HPP
