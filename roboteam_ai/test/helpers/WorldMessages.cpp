//
// Created by mrlukasbos on 14-1-19.
//

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/World.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <random>
#include <roboteam_ai/src/utilities/Constants.h>

namespace testhelpers {

double getRandomValue(double min, double max) {
    std::random_device rd;

    // engine
    std::mt19937 e2(rd());

    // set up distributions for generating random points
    std::uniform_real_distribution<> dist(min, max);

    return dist(e2);
}

// get a random field position
rtt::Vector2 getRandomFieldPosition(roboteam_msgs::GeometryFieldSize field) {
    auto randomX = getRandomValue(-(field.field_width/2), field.field_width/2);
    auto randomY = getRandomValue(-(field.field_length/2), field.field_length/2);
    return {randomX, randomY};
}

rtt::Vector2 getRandomVelocity() {
    rtt::Vector2 vector;
    do {
        auto xVel = getRandomValue(-rtt::ai::constants::MAX_VEL, rtt::ai::constants::MAX_VEL);
        auto yVel = getRandomValue(-rtt::ai::constants::MAX_VEL, rtt::ai::constants::MAX_VEL);
        vector = {xVel, yVel};
    } while (vector.length() >= rtt::ai::constants::MAX_VEL);
    return vector;
}

roboteam_msgs::WorldRobot generateRandomRobot(int id, roboteam_msgs::GeometryFieldSize field) {
    roboteam_msgs::WorldRobot robot;
    robot.id = (unsigned) id;
    robot.pos = getRandomFieldPosition(std::move(field));
    robot.angle = static_cast<float>(getRandomValue(rtt::ai::constants::MIN_ANGLE, rtt::ai::constants::MAX_ANGLE));
    robot.vel = getRandomVelocity();
    robot.w = static_cast<float>(getRandomValue(0, rtt::ai::constants::MAX_ANGULAR_VELOCITY));
    return robot;
}

roboteam_msgs::WorldBall generateRandomBall(roboteam_msgs::GeometryFieldSize field) {
    roboteam_msgs::WorldBall ball;
    ball.pos = getRandomFieldPosition(std::move(field));
    ball.vel = getRandomVelocity();
    ball.visible = 1;
    return ball;
}

// return a position right before a robot
// so close that we can say that the robot has the ball.
rtt::Vector2 getLocationRightBeforeRobot(roboteam_msgs::WorldRobot) {
    //TODO
    return {};
}

roboteam_msgs::WorldBall generateBallAtLocation(const rtt::Vector2 &loc) {
    roboteam_msgs::WorldBall ball;
    ball.pos = loc;
    ball.vel = rtt::Vector2(0, 0);
    ball.visible = 1;
    return ball;
}

std::vector<roboteam_msgs::WorldRobot> generateRandomRobots(int amount, const roboteam_msgs::GeometryFieldSize &field) {
    std::vector<roboteam_msgs::WorldRobot> robots;
    for (int i = 0; i<amount; i++) {
        robots.push_back(generateRandomRobot(i, field));
    }
    return robots;
}

// generate a world message with robots for both teams
roboteam_msgs::World getWorldMsg(int amountUs, int amountThem, bool withBall, const roboteam_msgs::GeometryFieldSize &field) {
    roboteam_msgs::World msg;
    msg.us = generateRandomRobots(amountUs, field);
    msg.them = generateRandomRobots(amountThem, field);
    if (withBall) msg.ball = generateRandomBall(field);
    return msg;
}

std::pair<roboteam_msgs::World, int> getWorldMsgWhereRobotHasBall(int amountUs, int amountThem, bool weHaveBall, roboteam_msgs::GeometryFieldSize field) {
    // first create a message with both teams and a ball
    roboteam_msgs::World msg = getWorldMsg(amountUs, amountThem, false, field);

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