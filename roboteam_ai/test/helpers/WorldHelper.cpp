//
// Created by mrlukasbos on 14-1-19.
//

#include "WorldHelper.h"
#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/World.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/World.h>
#include <random>
#include <roboteam_ai/src/utilities/Constants.h>

namespace testhelpers {

/*
 * Generate a random value in the uniform real distribution
 */
double WorldHelper::getRandomValue(double min, double max) {
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(min, max);
    return dist(e2);
}

/*
 * Generate a random position on a field
 */
rtt::Vector2 WorldHelper::getRandomFieldPosition(roboteam_msgs::GeometryFieldSize field) {
    auto randomX = getRandomValue(-(field.field_width/2), field.field_width/2);
    auto randomY = getRandomValue(-(field.field_length/2), field.field_length/2);
    return {randomX, randomY};
}

/*
 * Generate a random velocity which is lower than the maximum velocity
 */
rtt::Vector2 WorldHelper::getRandomVelocity() {
    auto xVel = getRandomValue(-rtt::ai::Constants::MAX_VEL(), rtt::ai::Constants::MAX_VEL());
    auto yVel = getRandomValue(-rtt::ai::Constants::MAX_VEL(), rtt::ai::Constants::MAX_VEL());
    rtt::Vector2 vector = {xVel, yVel};

    // limit the vector if needed
    if (vector.length() > rtt::ai::Constants::MAX_VEL()) {
        vector = vector.stretchToLength(rtt::ai::Constants::MAX_VEL());
    }
    return vector;
}

/*
 * Check if a world message is valid.
 * No robots should be on top of each other or the ball.
 */
bool WorldHelper::allPositionsAreValid(const roboteam_msgs::World &worldMsg, bool withBall) {
    std::vector<roboteam_msgs::WorldRobot> robots;
    robots.insert(robots.end(), worldMsg.us.begin(), worldMsg.us.end());
    robots.insert(robots.end(), worldMsg.them.begin(), worldMsg.them.end());

    std::vector<std::pair<int, rtt::Vector2>> robotPositions;
    for (int i = 0; i < robots.size(); i++) {
        robotPositions.emplace_back(std::make_pair(i, robots.at((unsigned long) i).pos));
    }

    // for each position, check all other positions and see if the distance is large enough.
    for (auto &pos : robotPositions) {
        for (auto &posToCompore : robotPositions) {

            // if the position is itself we don't need to do anything
            if (pos.first != posToCompore.first) {
                if (pos.second.dist((posToCompore.second)) < 2 * rtt::ai::Constants::ROBOT_RADIUS()) return false;
            }
        }

        if (withBall) {
            if (pos.second.dist(worldMsg.ball.pos) < rtt::ai::Constants::ROBOT_RADIUS() + rtt::ai::Constants::BALL_RADIUS()) {
                return false;
            }
        }
    }

    return true;
}

/*
 * Generate a robot on a random position
 */
roboteam_msgs::WorldRobot WorldHelper::generateRandomRobot(int id, roboteam_msgs::GeometryFieldSize field) {
    roboteam_msgs::WorldRobot robot;
    robot.id = (unsigned) id;
    robot.pos = getRandomFieldPosition(std::move(field));
    robot.angle = static_cast<float>(getRandomValue(rtt::ai::Constants::MIN_ANGLE(), rtt::ai::Constants::MAX_ANGLE()));
    robot.vel = getRandomVelocity();
    robot.w = static_cast<float>(getRandomValue(0, rtt::ai::Constants::MAX_ANGULAR_VELOCITY()));
    return robot;
}

/*
 * Generate a ball at a random position
 */
roboteam_msgs::WorldBall WorldHelper::generateRandomBall(roboteam_msgs::GeometryFieldSize field) {
    roboteam_msgs::WorldBall ball;
    ball.pos = getRandomFieldPosition(std::move(field));
    ball.vel = getRandomVelocity();
    ball.visible = 1;
    ball.existence = 99999;
    return ball;
}

/*
 * return a position right before a robot
 * so close that we can say that the robot has the ball.
 */
rtt::Vector2 WorldHelper::getLocationRightBeforeRobot(roboteam_msgs::WorldRobot robot) {
    rtt::Vector2 angleVector = rtt::Vector2(cos(robot.angle), sin(robot.angle));
    angleVector = angleVector.stretchToLength(rtt::ai::Constants::ROBOT_RADIUS());
    rtt::Vector2 robotPos = rtt::Vector2(robot.pos);
    return robotPos + angleVector;
}

/*
 * Generate a ball at a given location
 */
roboteam_msgs::WorldBall WorldHelper::generateBallAtLocation(const rtt::Vector2 &loc) {
    roboteam_msgs::WorldBall ball;
    ball.pos = loc;
    ball.vel = rtt::Vector2(0, 0);
    ball.visible = 1;
    ball.existence = 99999;
    return ball;
}

/*
 * Generate a certain amount of robots in a field at random positions
 */
std::vector<roboteam_msgs::WorldRobot> WorldHelper::generateRandomRobots(int amount, const roboteam_msgs::GeometryFieldSize &field) {
    std::vector<roboteam_msgs::WorldRobot> robots;
    for (int i = 0; i<amount; i++) {
        robots.push_back(generateRandomRobot(i, field));
    }
    return robots;
}

/*
 * Generate a world message for both teams
 */
roboteam_msgs::World WorldHelper::getWorldMsg(int amountUs, int amountThem, bool withBall, const roboteam_msgs::GeometryFieldSize &field) {
    roboteam_msgs::World msg;
    do {
        msg.us = generateRandomRobots(amountUs, field);
        msg.them = generateRandomRobots(amountThem, field);
        if (withBall) msg.ball = generateRandomBall(field);
    } while (! allPositionsAreValid(msg, true));
    return msg;
}

/*
 * Generate a world message for both teams where one of the robots has the ball.
 * Returns the id of the robot with the ball and and the world message
 */
std::pair<roboteam_msgs::World, int> WorldHelper::getWorldMsgWhereRobotHasBall(int amountUs, int amountThem, bool weHaveBall, roboteam_msgs::GeometryFieldSize field) {
    // first create a message with both teams and a ball
    roboteam_msgs::World msg;
    rtt::Vector2 ballLocation;
    int robotWithBallId;

    do {
        msg = getWorldMsg(amountUs, amountThem, false, field);
        // determine a list with robots of which one should have the ball
        std::vector<roboteam_msgs::WorldRobot> robots = weHaveBall ? msg.us : msg.them;
        std::random_shuffle(robots.begin(), robots.end());
        for (auto robot : robots) {
            ballLocation = getLocationRightBeforeRobot(robot);
            robotWithBallId = robot.id;
        }
        msg.ball = generateBallAtLocation(ballLocation);

    } while (!allPositionsAreValid(msg, false)); // WITHBALL must be set to false, since the ball is generated next to a robot!

    return std::make_pair(msg, robotWithBallId);
}

} // testhelpers