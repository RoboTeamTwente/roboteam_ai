//
// Created by emiel on 12-02-20.
//

#include <gtest/gtest.h>
#include <helpers/WorldHelper.h>

#include <world/World.hpp>

// TODO: Format the currently copy-pasted tests below properly together
TEST(worldTest, WhichRobotHasBallTest) {
    // set us to yellow
    rtt::SETTINGS.setYellow(true);

    // TODO: Extend test to also work in SIM environment making use of dribbler encoder and / or dedicated ballsensor
    // Base our test on vision information (instead of ballsensor/dribbler information)
    rtt::SETTINGS.setRobotHubMode(rtt::SETTINGS.BASESTATION);

    auto const& [_, worldInstance] = rtt::world::World::instance();

    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsYellow;
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsBlue;

    // Create robots yellow
    proto::WorldRobot roboty1;
    roboty1.set_id(1);
    roboty1.mutable_pos()->set_x(0.1);
    roboty1.set_angle(M_PI);

    proto::WorldRobot roboty2;
    roboty2.set_id(2);
    roboty2.mutable_pos()->set_x(0.5);
    roboty2.set_angle(M_PI);

    // Create robots blue
    proto::WorldRobot robotb1;
    robotb1.set_id(3);
    robotb1.mutable_pos()->set_x(0.05);
    robotb1.set_angle(M_PI);

    /* Setup :  Ball ..0.05.. robotb1(3) ..0.1.. roboty1(1) ..0.5.. roboty2(2) */

    // Create ball
    proto::WorldBall ball;
    ball.mutable_pos()->set_x(0.001);
    ball.mutable_pos()->set_y(0);
    ball.set_visible(true);

    // Add robots
    robotsYellow.Add()->CopyFrom(roboty1);
    robotsYellow.Add()->CopyFrom(roboty2);
    robotsBlue.Add()->CopyFrom(robotb1);

    /** Test 1 : Two yellow robots (ids 1 & 2), one blue robots (id = 3) and one ball **/
    proto::World msgBall;
    msgBall.mutable_yellow()->CopyFrom(robotsYellow);
    msgBall.mutable_blue()->CopyFrom(robotsBlue);
    msgBall.mutable_ball()->CopyFrom(ball);
    worldInstance->updateWorld(msgBall);
    auto world = worldInstance->getWorld();
    assert(world->getUs().size() == 2);
    assert(world->getThem().size() == 1);
    assert(world->getBall().has_value());

    std::optional<rtt::world::view::RobotView> robot;

    /** Test 1.1 : us */
    robot = world->whichRobotHasBall(rtt::world::Team::us);
    EXPECT_EQ((*robot)->getId(), 1);
    /** Test 1.2 : them */
    robot = world->whichRobotHasBall(rtt::world::Team::them);
    EXPECT_EQ((*robot)->getId(), 3);
    /** Test 1.3 : both */
    robot = world->whichRobotHasBall(rtt::world::Team::both);
    EXPECT_EQ((*robot)->getId(), 3);
}

TEST(worldTest, NoRobotHasBall) {
    // set us to yellow
    rtt::SETTINGS.setYellow(true);

    // Base our test on vision information (instead of ballsensor/dribbler information)
    rtt::SETTINGS.setRobotHubMode(rtt::SETTINGS.BASESTATION);

    auto const& [_, worldInstance] = rtt::world::World::instance();

    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsYellow;
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robotsBlue;

    // Create robots yellow
    proto::WorldRobot roboty1;
    roboty1.set_id(1);
    roboty1.mutable_pos()->set_x(1);
    roboty1.set_angle(M_PI);

    proto::WorldRobot roboty2;
    roboty2.set_id(2);
    roboty2.mutable_pos()->set_x(2);
    roboty2.set_angle(M_PI);

    // Create robots blue
    proto::WorldRobot robotb1;
    robotb1.set_id(3);
    robotb1.mutable_pos()->set_x(3);
    robotb1.set_angle(M_PI);

    /* Setup :  Ball ..1.. robotb1(3) ..2.. roboty1(1) ..3.. roboty2(2) */

    // Create ball
    proto::WorldBall ball;
    ball.mutable_pos()->set_x(0.001);
    ball.mutable_pos()->set_y(0);
    ball.set_visible(true);

    // Add robots
    robotsYellow.Add()->CopyFrom(roboty1);
    robotsYellow.Add()->CopyFrom(roboty2);
    robotsBlue.Add()->CopyFrom(robotb1);

    /** Test 1 : Two yellow robots (ids 1 & 2), one blue robots (id = 3) and one ball **/
    proto::World msgBall;
    msgBall.mutable_yellow()->CopyFrom(robotsYellow);
    msgBall.mutable_blue()->CopyFrom(robotsBlue);
    msgBall.mutable_ball()->CopyFrom(ball);
    worldInstance->updateWorld(msgBall);
    auto world = worldInstance->getWorld();
    assert(world->getUs().size() == 2);
    assert(world->getThem().size() == 1);
    assert(world->getBall().has_value());

    std::optional<rtt::world::view::RobotView> robot;


    /** Test 1.4 : both, all out of range */
    robot = world->whichRobotHasBall(rtt::world::Team::both);
    EXPECT_FALSE(robot.has_value());
}