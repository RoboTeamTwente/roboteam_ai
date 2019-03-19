//
// Created by robzelluf on 3/18/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/OffensiveCoach.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>
#include <algorithm>

namespace rtt {
namespace ai {
namespace coach {

TEST(OffensiveCoachTest, calculateNewPositionsTest) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 3;
    field.field_length = 5;
    Field::set_field(field);

    OffensiveCoach::calculateNewPositions();
    ASSERT_GT(OffensiveCoach::getOffensivePositions().size(), 1);
}

TEST(OffensiveCoachTest, calculateNewRobotPositions) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 3;
    field.field_length = 5;
    Field::set_field(field);

    OffensiveCoach::calculateNewPositions();
    ASSERT_GT(OffensiveCoach::getOffensivePositions().size(), 1);

    roboteam_msgs::World world;
    roboteam_msgs::WorldRobot robot1;
    robot1.id = 0;
    robot1.pos = Vector2({0, 0});

    roboteam_msgs::WorldRobot robot2;
    robot2.id = 1;
    robot2.pos = Vector2({3, 3});

    world.us.push_back(robot1);
    world.us.push_back(robot2);

    World::set_world(world);

    Vector2 robotPosition1 = OffensiveCoach::calculatePositionForRobot(std::make_shared<roboteam_msgs::WorldRobot>(robot1));
    Vector2 robotPosition2 = OffensiveCoach::calculatePositionForRobot(std::make_shared<roboteam_msgs::WorldRobot>(robot2));


    auto offensivePositions = OffensiveCoach::getOffensivePositions();
    std::vector<Vector2> offensivePositionVectors;
    for (auto& offensivePosition : offensivePositions) {
        offensivePositionVectors.push_back(offensivePosition.position);
    }

    ASSERT_TRUE(std::find(offensivePositionVectors.begin(), offensivePositionVectors.end(), robotPosition1) != offensivePositionVectors.end());
    ASSERT_TRUE(std::find(offensivePositionVectors.begin(), offensivePositionVectors.end(), robotPosition2) != offensivePositionVectors.end());
}

}
}
}
