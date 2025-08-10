#ifndef __SENSOR_BOX_HPP__
#define __SENSOR_BOX_HPP__

#include <cstdint>
#include "rover_can_lib/msgs/msg.hpp"

#if defined(ESP32)
#include "driver/twai.h"
#elif defined(__linux__)  // defined(ESP32)
#include <linux/can.h>
#endif  // defined(ESP32)

#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Msgs
{
    class SensorBox : public Msg
    {
      public:
        enum class eMsgID : uint8_t
        {
            NOT_USED = 0x00,
            OZONE = 0x01,
            HYDROGENE = 0x02,
            eLAST
        };

        struct sMsgData
        {
            float ozone float hydrogene
        };

        SensorBox()
        {
            data.ozone = 0.0f;
            data.hydrogene = 0.0f;
        }
        ~SensorBox() {}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t* msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::SENSOR_BOX)
            {
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::SensorBox::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
                case eMsgID::OZONE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.ozone);
                    break;

                case eMsgID::HYDROGENE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.hydrogene);
                    break;

                default:
                    LOG(WARN, "Unknown \"Message Specific Id\"");
                    return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t* msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::SENSOR_BOX;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::SensorBox::eMsgID)msgId_)
            {
                case eMsgID::OZONE:
                    Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.ozone, msg_);
                    break;

                case eMsgID::HYDROGENE:
                    Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.hydrogene, msg_);
                    break;

                default:
                    LOG(ERROR, "Shouldn't ever fall here, implementation error");
                    *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                    return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }
#elif defined(__linux__)  // defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const can_frame* msg_, rclcpp::Logger logger_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::SENSOR_BOX)
            {
                RCLCPP_ERROR(logger_,
                             "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::SensorBox::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
                case eMsgID::OZONE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.ozone, logger_);
                    break;

                case eMsgID::HYDROGENE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_,
                                                                                             &this->data.hydrogene,
                                                                                             logger_);
                    break;

                default:
                    RCLCPP_WARN(logger_, "Unknown \"Message Specific Id\"");
                    return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame* msg_, rclcpp::Logger logger_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::SENSOR_BOX;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::SensorBox::eMsgID)msgId_)
            {
                case eMsgID::OZONE:
                    Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.ozone, msg_);
                    break;

                case eMsgID::HYDROGENE:
                    Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.hydrogene, msg_);
                    break;

                default:
                    RCLCPP_ERROR(logger_, "Shouldn't ever fall here, implementation error");
                    *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                    return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode sendMsg(RoverCanLib::Constant::eDeviceId deviceID_, int canSocket_, rclcpp::Logger logger_)
        {
            can_frame canFrame;
            canFrame.can_id = (canid_t)deviceID_;

            static_assert((size_t)eMsgID::eLAST < UINT8_MAX);  // Make sure to not overflow counter
            for (uint8_t i = (uint8_t)eMsgID::NOT_USED + 1u; i < (uint8_t)eMsgID::eLAST; i++)
            {
                this->getMsg(i, &canFrame, logger_);
                if (write(canSocket_, &canFrame, sizeof(canFrame)) != sizeof(canFrame))
                {
                    RCLCPP_ERROR(logger_, "Error while sending error state msg");
                    return RoverCanLib::Constant::eInternalErrorCode::ERROR;
                }
            }

            return RoverCanLib::Constant::eInternalErrorCode::OK;
        }
#endif                    // defined(ESP32)

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}  // namespace RoverCanLib::Msgs

#endif  // __SENSOR_BOX_HPP__
