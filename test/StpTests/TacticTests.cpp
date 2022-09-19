//
// Created by ratoone on 12-03-20.
//

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stp/Tactic.h>

using namespace rtt::ai::stp;

class TestSkill : public Skill {
   protected:
    Status onUpdate(StpInfo const &info) noexcept override { return Status::Failure; }

    const char *getName() override { return "Test Skill"; }
};

class MockTactic : public Tactic {
   public:
    explicit MockTactic(bool emptyTactic) {
        if (!emptyTactic) {
            skills = rtt::collections::state_machine<Skill, Status, StpInfo>{TestSkill()};
        } else {
            skills = rtt::collections::state_machine<Skill, Status, StpInfo>{};
        }
    }

    std::optional<StpInfo> calculateInfoForSkill(const StpInfo &info) noexcept override { return StpInfo(); }

    MOCK_METHOD1(isTacticFailingMock, bool(const StpInfo &));

    bool isTacticFailing(const StpInfo &info) noexcept override { return isTacticFailingMock(info); }

    MOCK_METHOD1(shouldTacticResetMock, bool(const StpInfo &));

    bool shouldTacticReset(const StpInfo &info) noexcept override { return shouldTacticResetMock(info); }

    MOCK_METHOD0(isEndTacticMock, bool());

    bool isEndTactic() noexcept override { return isEndTacticMock(); }

    const char *getName() override { return "Mock Tactic"; }
};

// TODO: nice to have, but too much for this: a before for each test case; see junit for details
/** A non-end tactic will succeed when the state machine is finished.
 * As the state machine is empty, this only implies that the tactic is not an end tactic.
 */

TEST(TacticTests, nonEndTacticFinishedSuccessful) {
    MockTactic tactic(true);
    StpInfo info;
    info.setBall(rtt::world::view::BallView(nullptr));
    info.setField(rtt::world::Field());
    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world::robot::Robot(robot_proto);
    info.setRobot(rtt::world::view::RobotView(&robot));

    EXPECT_CALL(tactic, isEndTacticMock()).WillOnce(testing::Return(false));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Success, result);
}

/// The tactic will be waiting in the same condition as success, but it should be an end tactic
TEST(TacticTests, endTacticWaiting) {
    MockTactic tactic(true);
    StpInfo info;
    info.setBall(rtt::world::view::BallView(nullptr));
    info.setField(rtt::world::Field());
    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world::robot::Robot(robot_proto);
    info.setRobot(rtt::world::view::RobotView(&robot));

    EXPECT_CALL(tactic, isEndTacticMock()).WillOnce(testing::Return(true));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Waiting, result);
}

/// The tactic returns Failure if it didn't reach the end, and the failing condition is true.
TEST(TacticTests, endTacticFailingCondition) {
    MockTactic tactic(false);
    StpInfo info;
    info.setBall(rtt::world::view::BallView(nullptr));
    info.setField(rtt::world::Field());

    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world::robot::Robot(robot_proto);
    info.setRobot(rtt::world::view::RobotView(&robot));

    EXPECT_CALL(tactic, isTacticFailingMock(testing::_)).WillOnce(testing::Return(true));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Failure, result);
}

/// The tactic returns running if it's not an end tactic, and it didn't fail
TEST(TacticTests, isTacticRunningSuccessful) {
    MockTactic tactic(false);
    StpInfo info;
    info.setBall(rtt::world::view::BallView(nullptr));
    info.setField(rtt::world::Field());

    proto::WorldRobot robot_proto;
    robot_proto.set_id(1);
    std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;
    auto robot = rtt::world::robot::Robot(robot_proto);
    info.setRobot(rtt::world::view::RobotView(&robot));

    EXPECT_CALL(tactic, isTacticFailingMock(testing::_)).WillOnce(testing::Return(false));
    auto result = tactic.update(info);
    ASSERT_EQ(rtt::ai::stp::Status::Running, result);
}