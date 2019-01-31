#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>
#include <roboteam_ai/src/utilities/Coach.h>

namespace rtt {
namespace ai {
namespace coach {

TEST(CoachTest, robot_has_ball_tests) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 3, true, field);
    World::set_world(world.first);
    int robotThatHasBall = world.second;

    ASSERT_TRUE(Coach::doesRobotHaveBall(robotThatHasBall, true));
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall, false));
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall - 1, true));
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall + 1, true));

    ASSERT_EQ(Coach::whichRobotHasBall(true), robotThatHasBall);
    ASSERT_NE(Coach::whichRobotHasBall(false), robotThatHasBall);
}

} // coach
} // ai
} // rtt