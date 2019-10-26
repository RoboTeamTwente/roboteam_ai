//
// Created by thijs on 19-3-19.
//

#include "world/FutureWorld.h"
#include "world/History.h"
#include <utilities/Constants.h>

namespace rtt {
namespace ai {
namespace world {

void FutureWorld::updateFutureBall(BallPtr &ball, double time) {
    ball->setPos(ball->getPos() + ball->getVel() * time);
}

void FutureWorld::updateFutureRobot(RobotPtr &robot, double time) {
    robot->pos += robot->vel*time;
}

void FutureWorld::updateFutureWorld(WorldData &worldData, double time) {
    //TODO: take acceleration of the robot into account somehow :)

    // get a predicted future WorldState using linear extrapolation
    worldData.time = time;
    if(worldData.ball) {
        updateFutureBall(worldData.ball, time);
    }
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