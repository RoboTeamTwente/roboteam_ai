//
// Created by mrlukasbos on 20-3-19.
//

//
// Created by robzelluf on 1/29/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "../../src/skills/EnterFormation.h"
#include "../helpers/WorldHelper.h"

namespace rtt {
namespace ai {

TEST(FormationTest, formation_test) {
    robotDealer::RobotDealer::halt();

    // create a field and world
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 20;
    field.field_width = 10;
    rtt::ai::Field::set_field(field);
    rtt::ai::World::set_world(testhelpers::WorldHelper::getWorldMsg(2, 0, true, field));

    // generate a robot running the skill
    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("ROLE", "formationRobot");
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random, "EnterFormationTest", "formationRobot");
    rtt::ai::EnterFormation enterFormation("EnterFormationTest", properties);

    EXPECT_EQ(enterFormation.robotsInFormationMemory, 0);
    EXPECT_EQ(enterFormation.robotsInFormation.size(), 0);
    enterFormation.initialize();
    EXPECT_EQ(enterFormation.robotsInFormationMemory, 0);
    EXPECT_EQ(enterFormation.robotsInFormation.size(), 1);
    enterFormation.update();
    EXPECT_EQ(enterFormation.robotsInFormationMemory, 1);
    EXPECT_EQ(enterFormation.robotsInFormation.size(), 1);

    // get a position and store it
    Vector2 rememberPosition = enterFormation.getFormationPosition();

// generate a second robot with the same skill
    auto properties2 = std::make_shared<bt::Blackboard>();
    properties2->setString("ROLE", "formationRobot2");
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random, "EnterFormationTest2", "formationRobot2");
    rtt::ai::EnterFormation enterFormation2("EnterFormationTest2", properties2);

    EXPECT_EQ(enterFormation.robotsInFormationMemory, 1);
    EXPECT_EQ(enterFormation2.robotsInFormationMemory, 0);
    // the static vector should be the same
    EXPECT_EQ(enterFormation2.robotsInFormation.size(), 1);


    enterFormation2.initialize();
    EXPECT_EQ(enterFormation.robotsInFormationMemory, 1);
    EXPECT_EQ(enterFormation2.robotsInFormationMemory, 0);

    // after update it should be up-to-date again
    enterFormation2.update();
    enterFormation.update();
    EXPECT_EQ(enterFormation.robotsInFormationMemory, 2);
    EXPECT_EQ(enterFormation2.robotsInFormationMemory, 2);


    Vector2 newPosition = enterFormation.getFormationPosition();
    EXPECT_NE(rememberPosition, newPosition);

    // terminate the first node
    enterFormation.terminate(bt::Node::Status::Success);

    enterFormation2.update(); // propagate the changes (the fact that enterformation1 terminated)
    EXPECT_EQ(enterFormation2.robotsInFormationMemory, 1);

    newPosition = enterFormation2.getFormationPosition();
    EXPECT_EQ(rememberPosition, newPosition);

}


}
}
