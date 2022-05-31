#include "WorldHelper.h"

#include <utilities/Constants.h>

#include <random>

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
rtt::Vector2 WorldHelper::getRandomFieldPosition(const proto::SSL_GeometryFieldSize &field) {
    auto randomX = getRandomValue(-(field.field_length() / 2000.0), field.field_length() / 2000.0);
    auto randomY = getRandomValue(-(field.field_width() / 2000.0), field.field_width() / 2000.0);
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
bool WorldHelper::allPositionsAreValid(const proto::World &worldMsg, bool withBall) {
    std::vector<proto::WorldRobot> robots;
    robots.insert(robots.end(), worldMsg.yellow().begin(), worldMsg.yellow().end());
    robots.insert(robots.end(), worldMsg.blue().begin(), worldMsg.blue().end());

    std::vector<std::pair<int, rtt::Vector2>> robotPositions;
    for (unsigned int i = 0; i < robots.size(); i++) {
        robotPositions.emplace_back(std::make_pair(i, rtt::Vector2(robots.at((unsigned long)i).pos().x(), robots.at((unsigned long)i).pos().y())));
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
            if (pos.second.dist(rtt::Vector2(worldMsg.ball().pos().x(), worldMsg.ball().pos().y())) < rtt::ai::Constants::ROBOT_RADIUS() + rtt::ai::Constants::BALL_RADIUS()) {
                return false;
            }
        }
    }

    return true;
}

/*
 * Generate a robot on a random position
 */
proto::WorldRobot WorldHelper::generateRandomRobot(int id, proto::SSL_GeometryFieldSize field) {
    auto randomFieldPos = getRandomFieldPosition(std::move(field));
    auto randomVel = getRandomVelocity();

    auto robot = proto::WorldRobot();
    robot.set_id((unsigned)id);
    robot.mutable_pos()->set_x(randomFieldPos.x);
    robot.mutable_pos()->set_y(randomFieldPos.y);
    robot.set_angle(static_cast<float>(getRandomValue(rtt::ai::Constants::MIN_ANGLE(), rtt::ai::Constants::MAX_ANGLE())));
    robot.mutable_vel()->set_x(randomVel.x);
    robot.mutable_vel()->set_y(randomVel.x);
    robot.set_w(static_cast<float>(getRandomValue(0, rtt::ai::Constants::MAX_ANGULAR_VELOCITY())));
    return robot;
}

/*
 * Generate a ball at a random position
 */
proto::WorldBall *WorldHelper::generateRandomBall(proto::SSL_GeometryFieldSize field) {
    auto randomFieldPos = getRandomFieldPosition(std::move(field));
    auto randomVel = getRandomVelocity();

    auto ball = new proto::WorldBall;
    ball->mutable_pos()->set_x(randomFieldPos.x);
    ball->mutable_pos()->set_y(randomFieldPos.y);

    ball->mutable_vel()->set_x(randomVel.x);
    ball->mutable_vel()->set_y(randomVel.x);

    ball->set_visible(true);
    ball->set_area(99999);
    return ball;
}

/*
 * return a position right before a robot
 * so close that we can say that the robot has the ball.
 */
rtt::Vector2 WorldHelper::getLocationRightBeforeRobot(proto::WorldRobot robot) {
    rtt::Vector2 angleVector = rtt::Vector2(cos(robot.angle()), sin(robot.angle()));
    angleVector = angleVector.stretchToLength(rtt::ai::Constants::ROBOT_RADIUS());
    rtt::Vector2 robotPos = rtt::Vector2(robot.pos().x(), robot.pos().y());
    return robotPos + angleVector;
}

/*
 * Generate a ball at a given location
 */
proto::WorldBall WorldHelper::generateBallAtLocation(const rtt::Vector2 &loc) {
    proto::WorldBall ball;
    ball.mutable_vel()->set_x(0);
    ball.mutable_vel()->set_y(0);
    ball.mutable_pos()->set_x(loc.x);
    ball.mutable_pos()->set_y(loc.y);
    ball.set_visible(true);
    ball.set_area(99999);
    return ball;
}

/*
 * Generate a certain amount of robots in a field at random positions
 */
google::protobuf::RepeatedPtrField<proto::WorldRobot> WorldHelper::generateRandomRobots(int amount, const proto::SSL_GeometryFieldSize &field) {
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robots;
    for (int i = 0; i < amount; i++) {
        auto randombot = generateRandomRobot(i, field);
        auto botmsg = robots.Add();
        botmsg->CopyFrom(randombot);
    }
    return robots;
}

/*
 * Generate a world message for both teams
 */
proto::World WorldHelper::getWorldMsg(int amountYellow, int amountBlue, bool withBall, const proto::SSL_GeometryFieldSize &field) {
    proto::World msg;

    // Generate random robots and a ball and check if none are colliding
    // If there is a collision, generate new random robots
    do {
        auto randomBall = generateRandomBall(field);
        auto randomYellow = generateRandomRobots(amountYellow, field);
        auto randomBlue = generateRandomRobots(amountBlue, field);

        msg.mutable_yellow()->CopyFrom(randomYellow);
        msg.mutable_blue()->CopyFrom(randomBlue);

        if (withBall) msg.set_allocated_ball(randomBall);
    } while (!allPositionsAreValid(msg, withBall));

    return msg;
}

}  // namespace testhelpers