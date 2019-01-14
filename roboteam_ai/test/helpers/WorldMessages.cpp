//
// Created by mrlukasbos on 14-1-19.
//

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/World.h>

namespace testhelpers {

roboteam_msgs::WorldRobot generateRandomRobot(int id) {
    roboteam_msgs::WorldRobot robot;
    robot.id = (unsigned) id;
    robot.pos = rtt::Vector2(rand(), rand());
    robot.angle = rand();
    robot.vel = rtt::Vector2(rand(), rand());
    robot.w = rand();
    return robot;
}

roboteam_msgs::WorldBall generateRandomBall() {
    roboteam_msgs::WorldBall ball;
    ball.pos = rtt::Vector2(rand(), rand());
    ball.vel = rtt::Vector2(rand(), rand());
    ball.visible = true;
    return ball;
}

// return a position right before a robot
// so close that we can say that the robot has the ball.
rtt::Vector2 getLocationRightBeforeRobot(roboteam_msgs::WorldRobot) {
    //TODO
}

roboteam_msgs::WorldBall generateBallAtLocation(rtt::Vector2 loc) {
    roboteam_msgs::WorldBall ball;
    ball.pos = loc;
    ball.vel = rtt::Vector2(0, 0);
    ball.visible = true;
    return ball;
}

std::vector<roboteam_msgs::WorldRobot> generateRandomRobots(int amount) {
    for (int i = 0; i<amount; i++) {
        generateRandomRobot(i);
    }
}

// generate a world message with robots for both teams
roboteam_msgs::World getWorldMsg(int amountUs, int amountThem, bool withBall) {
    roboteam_msgs::World msg;
    msg.us = generateRandomRobots(amountUs);
    msg.them = generateRandomRobots(amountThem);
    if (withBall) msg.ball = generateRandomBall();
    return msg;
}

std::pair<roboteam_msgs::World, int> getWorldMsgWhereRobotHasBall(int amountUs, int amountThem, bool weHaveBall) {
    // first create a message with both teams and a ball
    roboteam_msgs::World msg = getWorldMsg(amountUs, amountThem, false);

    rtt::Vector2 ballLocation;
    int robotWithBallId;

    // determine a list with robots of which one should have the ball
    std::vector<roboteam_msgs::WorldRobot> robots = weHaveBall ? msg.us : msg.them;
    std::random_shuffle(robots.begin(), robots.end());
    for (auto robot : robots) {
        ballLocation = getLocationRightBeforeRobot(robot);
        robotWithBallId = robot.id;
    }

    msg.ball = generateBallAtLocation(ballLocation);
    return std::make_pair(msg, robotWithBallId);
}

} // testhelpers