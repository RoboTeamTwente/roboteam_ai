////
//// Created by mrlukasbos on 20-3-19.
////
//
<<<<<<< HEAD
//#include <gtest/gtest.h>
//#include <roboteam_ai/src/utilities/RobotDealer.h>
//#include <roboteam_ai/src/world/Field.h>
//#include "../../src/skills/Defend.h"
//#include "../helpers/WorldHelper.h"
//
//namespace rtt {
//namespace ai {
//
//TEST(Defendtest, defend_test) {
//    robotDealer::robotDealer->halt();
//
//    // create a field and world
//    roboteam_msgs::GeometryFieldSize field;
//    field.field_length = 20;
//    field.field_width = 10;
//    rtt::ai::world::field->set_field(field);
//    rtt::ai::world::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(2, 0, true, field));
//
//    // generate a robot running the skill
//    auto properties = std::make_shared<bt::Blackboard>();
//    properties->setString("ROLE", "defendRobot");
//    robotDealer::robotDealer->claimRobotForTactic(robotDealer::RobotType::RANDOM, "DefendTest", "defendRobot");
//    rtt::ai::Defend defend("DefendTest", properties);
//
//    EXPECT_EQ(defend.allDefendersMemory, 0);
//    EXPECT_EQ(defend.allDefenders.size(), 0);
//    defend.initialize();
//    EXPECT_EQ(defend.allDefendersMemory, 0);
//    EXPECT_EQ(defend.allDefenders.size(), 1);
//    defend.update();
//    EXPECT_EQ(defend.allDefendersMemory, 1);
//    EXPECT_EQ(defend.allDefenders.size(), 1);
//
//    // get a position and store it
//    Vector2 rememberPosition = defend.getDefensivePosition();
//
//// generate a second robot with the same skill
//    auto properties2 = std::make_shared<bt::Blackboard>();
//    properties2->setString("ROLE", "defendRobot2");
//    robotDealer::robotDealer->claimRobotForTactic(robotDealer::RobotType::RANDOM, "DefendTest2", "defendRobot2");
//    rtt::ai::Defend defend2("DefendTest2", properties2);
//
//    EXPECT_EQ(defend.allDefendersMemory, 1);
//    EXPECT_EQ(defend2.allDefendersMemory, 0);
//    // the static vector should be the same
//    EXPECT_EQ(defend2.allDefenders.size(), 1);
//
//
//    defend2.initialize();
//    EXPECT_EQ(defend.allDefendersMemory, 1);
//    EXPECT_EQ(defend2.allDefendersMemory, 0);
//
//    // after update it should be up-to-date again
//    defend2.update();
//    defend.update();
//    EXPECT_EQ(defend.allDefendersMemory, 2);
//    EXPECT_EQ(defend2.allDefendersMemory, 2);
//
//
//    Vector2 newPosition = defend.getDefensivePosition();
//    EXPECT_NE(rememberPosition, newPosition);
//
//    // terminate the first node
//    defend.terminate(bt::Node::Status::Success);
//
//    defend2.update(); // propagate the changes (the fact that enterformation1 terminated)
//    EXPECT_EQ(defend2.allDefendersMemory, 1);
//}
//
//
//}
//}
=======
// Created by mrlukasbos on 20-3-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "../../src/skills/Defend.h"
#include "../helpers/WorldHelper.h"

namespace rtt {
namespace ai {

TEST(Defendtest, defend_test) {
    robotDealer::RobotDealer::halt();

    // create a field and world
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 20;
    field.field_width = 10;
    rtt::ai::Field::set_field(field);
    rtt::ai::World::set_world(testhelpers::WorldHelper::getWorldMsg(3, 0, true, field));

    // generate a robot running the skill
    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("ROLE", "defendRobot");
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::RANDOM, "DefendTest", "defendRobot");
    rtt::ai::Defend defend("DefendTest", properties);

    EXPECT_EQ(defend.allDefendersMemory, 0);
    EXPECT_EQ(defend.allDefenders.size(), 0);
    defend.initialize();
    EXPECT_EQ(defend.allDefendersMemory, 0);
    EXPECT_EQ(defend.allDefenders.size(), 1);
    defend.update();
    EXPECT_EQ(defend.allDefendersMemory, 1);
    EXPECT_EQ(defend.allDefenders.size(), 1);

    // get a position and store it
    Vector2 rememberPosition = defend.getDefensivePosition();

// generate a second robot with the same skill
    auto properties2 = std::make_shared<bt::Blackboard>();
    properties2->setString("ROLE", "defendRobot2");
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::RANDOM, "DefendTest2", "defendRobot2");
    rtt::ai::Defend defend2("DefendTest2", properties2);

    EXPECT_EQ(defend.allDefendersMemory, 1);
    EXPECT_EQ(defend2.allDefendersMemory, 0);
    // the static vector should be the same
    EXPECT_EQ(defend2.allDefenders.size(), 1);


    defend2.initialize();
    EXPECT_EQ(defend.allDefendersMemory, 1);
    EXPECT_EQ(defend2.allDefendersMemory, 0);

    // after update it should be up-to-date again
    defend2.update();
    defend.update();
    EXPECT_EQ(defend.allDefendersMemory, 2);
    EXPECT_EQ(defend2.allDefendersMemory, 2);


    Vector2 newPosition = defend.getDefensivePosition();
    EXPECT_NE(rememberPosition, newPosition);

    // terminate the first node
    defend.terminate(bt::Node::Status::Success);

    defend2.update(); // propagate the changes (the fact that enterformation1 terminated)
    EXPECT_EQ(defend2.allDefendersMemory, 1);

    newPosition = defend2.getDefensivePosition();
    EXPECT_EQ(rememberPosition, newPosition);

}


}
}
>>>>>>> origin/development
