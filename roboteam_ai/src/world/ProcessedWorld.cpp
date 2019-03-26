//
// Created by thijs on 19-3-19.
//

#include "ProcessedWorld.h"

namespace rtt {
namespace ai {
namespace world {

void ProcessedWorld::update() {

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
    ball.pos += ball.vel * time;
}

void ProcessedWorld::updateFutureRobot(Robot &robot, double time) {
    robot.pos += robot.vel * time;
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