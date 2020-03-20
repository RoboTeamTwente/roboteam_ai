//
// Created by ratoone on 12-03-20.
//

#include <stp/Tactic.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace rtt::ai::stp;

class MockTactic : public Tactic{
public:
    StpInfo calculateInfoForSkill(const StpInfo &info) noexcept override {
        return StpInfo();
    }

    void onInitialize() noexcept override {}

    void onUpdate(const Status &status) noexcept override {}

    void onTerminate() noexcept override {}

    MOCK_METHOD1(isTacticFailingMock, bool(const StpInfo&));

    bool isTacticFailing(const StpInfo &info) noexcept override {
        return isTacticFailingMock(info);
    }

    MOCK_METHOD1(shouldTacticResetMock, bool(const StpInfo&));

    bool shouldTacticReset(const StpInfo &info) noexcept override {
        return shouldTacticResetMock(info);
    }

    MOCK_METHOD0(isEndTacticMock, bool());

    bool isEndTactic() noexcept override {
        return isEndTacticMock();
    }
};

//TODO: nice to have, but too much for this: a before for each test case; see junit for details
/** A non-end tactic will succeed when the state machine is finished.
 * As the state machine is empty, this only implies that the tactic is not an end tactic.
 */

TEST(TacticTests, nonEndTacticFinishedSuccessful){
    MockTactic tactic;
    StpInfo info;
    info.setBall(rtt::world_new::view::BallView(nullptr));
    info.setField(rtt::ai::world::Field());
    info.setRobot(rtt::world_new::view::RobotView(nullptr));

    EXPECT_CALL(tactic, isEndTacticMock()).WillOnce(testing::Return(false));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Success, result);
}

/** The tactic returns Failure if it didn't reach the end, and the failing condition is
 * true. As the state machine is empty, this implies that this should be an end tactic
 * (as it cannot succeed)
 */
TEST(TacticTests, endTacticFailingCondition){
    MockTactic tactic;
    StpInfo info;
    info.setBall(rtt::world_new::view::BallView(nullptr));
    info.setField(rtt::ai::world::Field());

    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    robot_proto.mutable_pos()->set_x(0.1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world_new::robot::Robot(updateMap, robot_proto);
    info.setRobot(rtt::world_new::view::RobotView(&robot));

    EXPECT_CALL(tactic, isEndTacticMock()).WillOnce(testing::Return(true));
    EXPECT_CALL(tactic, isTacticFailingMock(testing::_)).WillOnce(testing::Return(true));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Failure, result);
}

/// The tactic returns running if it's not an end tactic, and it didn't fail
TEST(TacticTests, isTacticRunningSuccessful){
    MockTactic tactic;
    StpInfo info;
    info.setBall(rtt::world_new::view::BallView(nullptr));
    info.setField(rtt::ai::world::Field());

    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    robot_proto.mutable_pos()->set_x(0.1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world_new::robot::Robot(updateMap, robot_proto);
    info.setRobot(rtt::world_new::view::RobotView(&robot));

    EXPECT_CALL(tactic, isEndTacticMock()).WillOnce(testing::Return(true));
    EXPECT_CALL(tactic, isTacticFailingMock(testing::_)).WillOnce(testing::Return(false));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Running, result);
}