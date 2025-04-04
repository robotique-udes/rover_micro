#include <gtest/gtest.h>

#include <rover_can2/msgs/test_msg.hpp>

// =============================================================================
// Helpers
// =============================================================================
namespace TestMsg
{

}  // namespace TestMsg

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_ROVER_CAN2_Msg, Construction)
{
    RoverCan2::Msgs::TestMsg msg;
}

TEST(SUITE_ROVER_CAN2_Msg, IdReporting)
{
    RoverCan2::Msgs::TestMsg msg;
    GTEST_ASSERT_TRUE(msg.getMsgId() == RoverCan2::Constant::eMsgId::TEST_MSG);
}

TEST(SUITE_ROVER_CAN2_Msg, InitToZero)
{
    RoverCan2::Msgs::TestMsg msg;
    GTEST_ASSERT_TRUE(msg.getMsgId() == RoverCan2::Constant::eMsgId::TEST_MSG);

    GTEST_ASSERT_TRUE(msg.data().closeLoop == false);
    GTEST_ASSERT_TRUE(msg.data().cmd == 0.0F);
}

TEST(SUITE_ROVER_CAN2_Msg, MsgLoading)
{
    bool closeLoop = true;
    std::array<uint8_t, 8U> data = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
                                    TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
                                    closeLoop};

    RoverCan2::CanMsg canMsg(RoverCan2::Constant::eDeviceId::NOT_SET, data.data(), 3U);

    RoverCan2::Msgs::TestMsg msg;
    GTEST_ASSERT_TRUE(msg.loadMsg(canMsg) == RoverCan2::Msgs::eLoadMsgCode::SUCCESS_COMPLETE);
    GTEST_ASSERT_TRUE(msg.getMsgId() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msg.getMsgContentCount() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST));
    GTEST_ASSERT_TRUE(msg.data().closeLoop == true);
    GTEST_ASSERT_TRUE(msg.data().cmd == 0.0F);
}

TEST(SUITE_ROVER_CAN2_Msg, MsgBuilding)
{
    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = true;

    std::optional<const RoverCan2::CanMsg> canMsg
        = msg.getCanMsg(TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP));
    GTEST_ASSERT_TRUE(canMsg.has_value() == true);
    GTEST_ASSERT_TRUE(canMsg.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP));
    GTEST_ASSERT_TRUE(canMsg.value().dataLength == 3U);
    GTEST_ASSERT_TRUE(static_cast<bool>(canMsg.value().msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)])
                      == true);
}

TEST(SUITE_ROVER_CAN2_Msg, MsgBuilding_ZeroInit)
{
    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = true;

    std::optional<const RoverCan2::CanMsg> canMsg = msg.getCanMsg(TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD));
    GTEST_ASSERT_TRUE(canMsg.has_value() == true);
    GTEST_ASSERT_TRUE(canMsg.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD));
    GTEST_ASSERT_TRUE(canMsg.value().dataLength == 6U);
    GTEST_ASSERT_TRUE(static_cast<float>(canMsg.value().msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)])
                      == 0.0F);
}

TEST(SUITE_ROVER_CAN2_Msg, MsgBuilding_InvalidID)
{
    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = true;

    std::optional<const RoverCan2::CanMsg> canMsg = msg.getCanMsg(TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST));
    GTEST_ASSERT_TRUE(canMsg.has_value() == false);
}
