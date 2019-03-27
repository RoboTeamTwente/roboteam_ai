//
// Created by thijs on 19-3-19.
//

#include "ProcessedWorld.h"
#include "roboteam_ai/src/utilities/Constants.h"

namespace rtt {
namespace ai {
namespace world {

ProcessedWorld processedWorldObj;
ProcessedWorld* processedWorld = &processedWorldObj;

Ball ProcessedWorld::updateBallPosition(WorldData &worldData) {
    //TODO:: :: WTF IS THIS CRAP??
    auto _world = worldData;
    Ball newBall = _world.ball;
    if (_world.ball.visible) {
        return newBall;
    }
    // we set the ball velocity to 0
    newBall.vel.x = 0;
    newBall.vel.y = 0;
    // if the ball was dribbled in the previous world_state we set its position to be in front of the robot that was dribbling it
    if (! OurBotsBall.empty() || ! TheirBotsBall.empty()) { // check if it was dribbled in last world state
        double maxDist = 100;
        int bestId = - 1;
        bool ourTeam = true;
        for (auto bot: OurBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = true;
            }
        }
        for (auto bot: TheirBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = false;
            }
        }
        if (bestId == - 1) {
            return newBall;
        }
        // I can't use getRobotForId because of deadlocking and because i need to use the world in the packet
        Robot robot;
        const std::vector<Robot> &robots = ourTeam ? _world.us : _world.them;
        for (const auto &bot : robots) {
            if (bot.id == bestId) {
                robot = bot;
                break;
            }
        }
        // put the ball in front of the centre robot that is dribbling it.
        Vector2 ballPos = Vector2(robot.pos)
                + Vector2(Constants::CENTRE_TO_FRONT() + Constants::BALL_RADIUS(), 0).rotate(robot.angle);
        newBall.pos = ballPos;
    }
        // else (not visible but not dribbled), we put it at its previous position
    else {
        newBall.pos = world.ball.pos;
    }
    return newBall;

}

void ProcessedWorld::updateBallPossession(WorldData &worldData) {

}

void ProcessedWorld::update(WorldData worldData) {
    updateBallPosition(worldData);
    updateBallPossession(worldData);

}

ProcessedWorld::RobotPtr ProcessedWorld::getRobotClosestToPoint(
        const Vector2 &point, std::vector<Robot> &robots) {

    if (robots.empty())
        return RobotPtr(nullptr);

    Robot* closestRobot = &robots[0];
    for (auto &robot : robots) {
        if ((robot.pos - point).length() < (closestRobot->pos - point).length()) {
            closestRobot = &robot;
        }
    }
    return RobotPtr(closestRobot);
}

void ProcessedWorld::updateFutureBall(Ball &ball, double time) {
    ball.pos += ball.vel*time;
}

void ProcessedWorld::updateFutureRobot(Robot &robot, double time) {
    robot.pos += robot.vel*time;
}

void ProcessedWorld::updateFutureWorld(WorldData &worldData, double time) {
    worldData.time = time;
    updateFutureBall(worldData.ball, time);
    for (auto &robot : worldData.us) {
        updateFutureRobot(robot, time);
    }
    for (auto &robot : worldData.them) {
        updateFutureRobot(robot, time);
    }
}

}
}
}