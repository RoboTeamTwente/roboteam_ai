//
// Created by thijs on 19-3-19.
//

#include "World.h"
#include "ProcessedWorld.h"

namespace rtt {
namespace ai {
namespace world {

bool World::weHaveRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    return !worldData.us.empty();
}

void World::setWorld(const roboteam_msgs::World &worldMsg) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldMsg = worldMsg;
    World::worldData = WorldData(worldMsg);
}

const roboteam_msgs::World &World::getWorldMsg() {
    std::lock_guard<std::mutex> lock(worldMsgMutex);

    return worldMsg;
}

const roboteam_msgs::WorldBall &World::getBallMsg() {
    std::lock_guard<std::mutex> lock(worldMsgMutex);

    return worldMsg.ball;
}

const WorldData &World::getWorld() {
    std::lock_guard<std::mutex> lock(worldMutex);

    return worldData;
}

std::shared_ptr<Ball> World::getBall() {
    Ball ballCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballCopy = worldData.ball;
    }
    if (ballCopy.exists != 0)
        return std::make_shared<Ball>(ballCopy);

    return BallPtr(nullptr);

}

std::shared_ptr<Robot> World::getRobotForId(int id, bool ourTeam) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    const std::vector<Robot> &robots = ourTeam ? worldCopy.us : worldCopy.them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return std::make_shared<Robot>(bot);
        }
    }
    return RobotPtr(nullptr);
}

std::vector<std::shared_ptr<Robot>> World::getAllRobots() {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    std::vector<RobotPtr> allRobots;
    for (auto &robot : worldCopy.us)
        allRobots.push_back(std::make_shared<Robot>(robot));
    for (auto &robot : worldCopy.them)
        allRobots.push_back(std::make_shared<Robot>(robot));
    return allRobots;
}

World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots) {
    std::vector<Robot> robotsCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        switch (whichRobots) {
        case OUR_ROBOTS:
            robotsCopy = worldData.us;
            break;
        case THEIR_ROBOTS:
            robotsCopy = worldData.them;
            break;
        case ALL_ROBOTS:
        default:
            robotsCopy = worldData.us;
            robotsCopy.insert(robotsCopy.end(), worldData.them.begin(), worldData.them.end());
            break;
        }
    }

    return processedWorld->getRobotClosestToPoint(point, robotsCopy);
}

World::RobotPtr World::getRobotClosestToRobot(int id, bool ourTeam, WhichRobots whichRobots) {
    std::vector<Robot> robotsCopy;
    Vector2 robotPos = Vector2();
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        robotsCopy = ourTeam ? worldData.us : worldData.them;
        for (auto it = robotsCopy.begin(); it != robotsCopy.end(); it++) {
            if (it->id == id) {
                robotPos = it->pos;
                robotsCopy.erase(it);
            }
        }
        robotsCopy.insert(robotsCopy.end(),
                ourTeam ? worldData.them.begin() : worldData.us.begin(),
                ourTeam ? worldData.them.end() : worldData.us.end());
    }

    return processedWorld->getRobotClosestToPoint(robotPos, robotsCopy);
}

World::RobotPtr World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos = Vector2();
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballPos = worldData.ball.pos;
    }

    return getRobotClosestToPoint(ballPos, whichRobots);
}

bool World::robotHasBall(int id, bool ourTeam, double maxDist) {
    return ourTeam ? ourRobotHasBall(id, maxDist) : theirRobotHasBall(id, maxDist);
}

bool World::ourRobotHasBall(int id, double maxDist) {
    //TODO:
    return false;
}

bool World::theirRobotHasBall(int id, double maxDist) {
    //TODO:
    return false;
}

int World::whichRobotHasBall(WhichRobots whichRobots) {
    //TODO:
    return 0;
}

const WorldData World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    processedWorld->updateFutureWorld(worldCopy, time);
    return worldCopy;
}

//ProcessedWorld* World::getProcessedWorld() {
//    return processedWorld;
//}
//h
} //world
} //ai
} //rtt