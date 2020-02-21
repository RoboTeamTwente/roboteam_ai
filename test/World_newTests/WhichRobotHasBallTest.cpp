//
// Created by emiel on 12-02-20.
//

#include <gtest/gtest.h>
#include <test/helpers/FieldHelper.h>
#include <test/helpers/WorldHelper.h>
#include <include/roboteam_ai/world_new/World.hpp>

TEST(World_newTest, WhichRobotHasBallTest) {

    // set us to yellow
    rtt::SETTINGS.setYellow(true);

    auto worldInstance = rtt::world_new::World::instance();



    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsYellow;
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsBlue;

    // Create robots yellow
    proto::WorldRobot roboty1;
    roboty1.set_id(1);
    roboty1.mutable_pos()->set_x(0.1);
    proto::WorldRobot roboty2;
    roboty2.set_id(2);
    roboty2.mutable_pos()->set_x(0.5);

    // Create robots blue
    proto::WorldRobot robotb1;
    robotb1.set_id(3);
    robotb1.mutable_pos()->set_x(0.05);

    /* Setup :  Ball ..0.05.. robotb1(3) ..0.1.. roboty1(1) ..0.5.. roboty2(2) */

    // Create ball
    proto::WorldBall ball;
    ball.mutable_pos()->set_x(0);
    ball.mutable_pos()->set_y(0);
    ball.set_visible(true);

    // Add robots
    robotsYellow.Add()->CopyFrom(roboty1);
    robotsYellow.Add()->CopyFrom(roboty2);
    robotsBlue.Add()->CopyFrom(robotb1);

    /** Test 1 : no ball. No robot closest */
    proto::World msgNoBall;
    msgNoBall.mutable_yellow()->CopyFrom(robotsYellow);
    msgNoBall.mutable_blue()->CopyFrom(robotsBlue);
    worldInstance->updateWorld(msgNoBall);

    auto world = worldInstance->getWorld();
    assert(world->getUs().size() == 2);
    assert(world->getThem().size() == 1);
    assert(!world->getBall().has_value());
    std::optional<rtt::world_new::view::RobotView> robot = world->whichRobotHasBall(rtt::world_new::Team::us);
    EXPECT_TRUE(!robot.has_value());


    /** Test 2 : Two yellow robots (1 & 2), two blue robots (3 & 4), one ball **/
    proto::World msgBall;
    msgBall.mutable_yellow()->CopyFrom(robotsYellow);
    msgBall.mutable_blue()->CopyFrom(robotsBlue);
    msgBall.mutable_ball()->CopyFrom(ball);
    worldInstance->updateWorld(msgBall);
    world = worldInstance->getWorld();
    assert(world->getUs().size() == 2);
    assert(world->getThem().size() == 1);
    assert(world->getBall().has_value());

    /** Test 2.1 : us */
    robot = world->whichRobotHasBall(rtt::world_new::Team::us, 1.0);
    EXPECT_EQ((*robot)->getId(), 1);
    /** Test 2.2 : them */
    robot = world->whichRobotHasBall(rtt::world_new::Team::them, 1.0);
    EXPECT_EQ((*robot)->getId(), 3);
    /** Test 2.3 : both */
    robot = world->whichRobotHasBall(rtt::world_new::Team::both, 1.0);
    EXPECT_EQ((*robot)->getId(), 3);
    /** Test 2.4 : both, all out of range */
    robot = world->whichRobotHasBall(rtt::world_new::Team::both, 0.001);
    EXPECT_FALSE(robot.has_value());


}