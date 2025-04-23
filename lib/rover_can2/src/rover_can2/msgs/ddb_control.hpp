#ifndef DDB_CONTROL_HPP
#define DDB_CONTROL_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(DdbControl_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class DdbControl : public Msg<DdbControl>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            BANK_0_CH_0_FREQ,
            BANK_0_CH_0_DUTY,
            BANK_0_CH_1_FREQ,
            BANK_0_CH_1_DUTY,
            BANK_0_CH_2_FREQ,
            BANK_0_CH_2_DUTY,
            BANK_0_CH_3_FREQ,
            BANK_0_CH_3_DUTY,
            BANK_1_CH_0_ON,
            BANK_1_CH_1_ON,
            BANK_1_CH_2_ON,
            BANK_1_CH_3_ON,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float bank0_ch0_freq;
            float bank0_ch0_duty;
            float bank0_ch1_freq;
            float bank0_ch1_duty;
            float bank0_ch2_freq;
            float bank0_ch2_duty;
            float bank0_ch3_freq;
            float bank0_ch3_duty;
            bool bank1_ch0_on;
            bool bank1_ch1_on;
            bool bank1_ch2_on;
            bool bank1_ch3_on;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::BANK_0_CH_0_FREQ, eMsgContentID::BANK_0_CH_0_DUTY, eMsgContentID::BANK_0_CH_1_FREQ, eMsgContentID::BANK_0_CH_1_DUTY, eMsgContentID::BANK_0_CH_2_FREQ, eMsgContentID::BANK_0_CH_2_DUTY, eMsgContentID::BANK_0_CH_3_FREQ, eMsgContentID::BANK_0_CH_3_DUTY, eMsgContentID::BANK_1_CH_0_ON, eMsgContentID::BANK_1_CH_1_ON, eMsgContentID::BANK_1_CH_2_ON, eMsgContentID::BANK_1_CH_3_ON};

      public:
        DdbControl();

        eLoadMsgCode _loadMsg(const CanMsg& msg_);
        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const;
        uint8_t _getMsgContentCount(void) const;
        sMsgData& data(void);
        const sMsgData& getData(void) const;

      private:
        sMsgData _data;
    };

    DdbControl::DdbControl():
        Msg(Constant::eMsgId::DDB_CONTROL)
    {
        _data.bank0_ch0_freq = static_cast<decltype(_data.bank0_ch0_freq)>(0);
        _data.bank0_ch0_duty = static_cast<decltype(_data.bank0_ch0_duty)>(0);
        _data.bank0_ch1_freq = static_cast<decltype(_data.bank0_ch1_freq)>(0);
        _data.bank0_ch1_duty = static_cast<decltype(_data.bank0_ch1_duty)>(0);
        _data.bank0_ch2_freq = static_cast<decltype(_data.bank0_ch2_freq)>(0);
        _data.bank0_ch2_duty = static_cast<decltype(_data.bank0_ch2_duty)>(0);
        _data.bank0_ch3_freq = static_cast<decltype(_data.bank0_ch3_freq)>(0);
        _data.bank0_ch3_duty = static_cast<decltype(_data.bank0_ch3_duty)>(0);
        _data.bank1_ch0_on = static_cast<decltype(_data.bank1_ch0_on)>(0);
        _data.bank1_ch1_on = static_cast<decltype(_data.bank1_ch1_on)>(0);
        _data.bank1_ch2_on = static_cast<decltype(_data.bank1_ch2_on)>(0);
        _data.bank1_ch3_on = static_cast<decltype(_data.bank1_ch3_on)>(0);
    }

    eLoadMsgCode DdbControl::_loadMsg(const CanMsg& msg_)
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
            LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                      "Missmatch between received message and local message definition. Received msgContentId: (%u), "
                      "expected lower than (%u) and none zero",
                      TO_UNDERLYING(msgContentId),
                      TO_UNDERLYING(eMsgContentID::eLAST));
            return eLoadMsgCode::ERROR_MISSMATCH;
        }

        bool success = false;
        switch (msgContentId)
        {
            case eMsgContentID::BANK_0_CH_0_FREQ:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch0_freq);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_0_FREQ: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_0_DUTY:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch0_duty);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_0_DUTY: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_1_FREQ:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch1_freq);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_1_FREQ: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_1_DUTY:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch1_duty);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_1_DUTY: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_2_FREQ:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch2_freq);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_2_FREQ: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_2_DUTY:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch2_duty);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_2_DUTY: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_3_FREQ:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch3_freq);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_3_FREQ: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_0_CH_3_DUTY:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank0_ch3_duty);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_0_CH_3_DUTY: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_1_CH_0_ON:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank1_ch0_on);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_1_CH_0_ON: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_1_CH_1_ON:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank1_ch1_on);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_1_CH_1_ON: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_1_CH_2_ON:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank1_ch2_on);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_1_CH_2_ON: %s",
                          success ? "success" : "failed");
                break;

            case eMsgContentID::BANK_1_CH_3_ON:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.bank1_ch3_on);
                LOG_DEBUG(Logger::Nodes::DdbControl_msg,
                          "switch (msgContentId) case eMsgContentID::BANK_1_CH_3_ON: %s",
                          success ? "success" : "failed");
                break;

            default:
                return eLoadMsgCode::ERROR_IMPLEMENTATION;
        }

        if (!success)
        {
            return eLoadMsgCode::ERROR_MISSMATCH;
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

    std::optional<CanMsg> DdbControl::_getCanMsg(const uint8_t msgContentId_) const
    {
        eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

        if (!VALID_MSG_IDS.contains(msgContentID))
        {
            return std::nullopt;
        }

        CanMsg msg_;
        switch (static_cast<eMsgContentID>(msgContentId_))
        {
            case eMsgContentID::BANK_0_CH_0_FREQ:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch0_freq, msg_);
                break;

            case eMsgContentID::BANK_0_CH_0_DUTY:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch0_duty, msg_);
                break;

            case eMsgContentID::BANK_0_CH_1_FREQ:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch1_freq, msg_);
                break;

            case eMsgContentID::BANK_0_CH_1_DUTY:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch1_duty, msg_);
                break;

            case eMsgContentID::BANK_0_CH_2_FREQ:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch2_freq, msg_);
                break;

            case eMsgContentID::BANK_0_CH_2_DUTY:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch2_duty, msg_);
                break;

            case eMsgContentID::BANK_0_CH_3_FREQ:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch3_freq, msg_);
                break;

            case eMsgContentID::BANK_0_CH_3_DUTY:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank0_ch3_duty, msg_);
                break;

            case eMsgContentID::BANK_1_CH_0_ON:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank1_ch0_on, msg_);
                break;

            case eMsgContentID::BANK_1_CH_1_ON:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank1_ch1_on, msg_);
                break;

            case eMsgContentID::BANK_1_CH_2_ON:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank1_ch2_on, msg_);
                break;

            case eMsgContentID::BANK_1_CH_3_ON:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.bank1_ch3_on, msg_);
                break;

            case eMsgContentID::eLAST:
                return std::nullopt;
        }

        return msg_;
    }

    uint8_t DdbControl::_getMsgContentCount(void) const
    {
        return TO_UNDERLYING(eMsgContentID::eLAST);
    }

    DdbControl::sMsgData& DdbControl::data(void)
    {
        return _data;
    }
    
    const DdbControl::sMsgData& DdbControl::getData(void) const
    {
        return static_cast<const DdbControl::sMsgData&>(_data);
    }
}  // namespace RoverCan2::Msgs

#endif  // DDB_CONTROL_HPP
