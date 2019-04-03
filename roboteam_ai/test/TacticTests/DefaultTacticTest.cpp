//
// Created by mrlukasbos on 11-1-19.
//


#include <gtest/gtest.h>
#include <roboteam_ai/src/bt/tactics/DefaultTactic.h>
#include <roboteam_ai/src/Switches.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include <roboteam_ai/src/utilities/World.h>
#include "../../src/utilities/RobotDealer.h"

TEST(DefaultTacticTest, it_takes_robots) {
    using dealer = robotDealer::RobotDealer;
    using robotType = robotDealer::RobotType;
    // Make sure that there is a world and that it is empty
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2, robot3, robot4, robot5, robot6, robot7, robot8;
    rtt::ai::World::set_world(worldMsg);
    dealer::removeTactic("free"); // This is necessary because previous tests create free robots
    ASSERT_TRUE(dealer::getAvailableRobots().empty());

    //fill the world
    robot1.id=1;
    robot2.id=2;
    robot3.id=3;
    robot4.id=4;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    worldMsg.us.push_back(robot3);
    worldMsg.us.push_back(robot4);

    rtt::ai::World::set_world(worldMsg);

    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();

    BTFactory::makeTrees();

    auto strategy = BTFactory::getTree("randomStrategy");
    strategy->tick();

    auto repeater = strategy->GetRoot();
    EXPECT_EQ(repeater->node_name(), "Repeater");

    auto tacticNode = repeater->getChildren().at(0);

    // here the tactics get initialized
    ASSERT_EQ(tacticNode->node_name(), "randomTactic");
    ASSERT_EQ(tacticNode->getChildren().size(), 1); // it has one child (which is parallelsequence)
    ASSERT_EQ(dealer::getAvailableRobots().size(), 3);

    strategy->tick();

    // halttactic is waiting because not enough robots are available
    ASSERT_EQ(tacticNode->getStatus(), bt::Node::Status::Waiting);

    // add more robots to the world
    robot4.id=4;
    robot5.id=5;
    robot6.id=6;
    robot7.id=7;
    worldMsg.us.push_back(robot4);
    worldMsg.us.push_back(robot5);
    worldMsg.us.push_back(robot6);
    worldMsg.us.push_back(robot7);
    rtt::ai::World::set_world(worldMsg);

    // now there are enough robots
    ASSERT_EQ(dealer::getAvailableRobots().size(), 7);
    ASSERT_EQ(tacticNode->getStatus(), bt::Node::Status::Waiting);

    strategy->tick();
    ASSERT_EQ(tacticNode->getStatus(), bt::Node::Status::Running);

    // now all robots should be claimed for randomTactic and randomTactic should start Running
    ASSERT_EQ(dealer::getAvailableRobots().size(), 0);
    ASSERT_EQ(dealer::getClaimedRobots().at("randomTactic").size(), 7); //robotdealer should say randomTactic has claimed 7 robots

    strategy->tick();
    ASSERT_EQ(tacticNode->getStatus(), bt::Node::Status::Running);

    strategy->terminate(bt::Node::Status::Running);
    ASSERT_EQ(tacticNode->getStatus(), bt::Node::Status::Failure);
    ASSERT_EQ(strategy->getStatus(), bt::Node::Status::Failure);

    // the robots should be available again.
    ASSERT_EQ(dealer::getAvailableRobots().size(), 7);

}