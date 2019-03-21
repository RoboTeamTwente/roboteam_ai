//
// Created by thijs on 19-3-19.
//

#include "World.h"

namespace rtt {
namespace ai {
namespace world {

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

    return std::make_shared<Ball>(ballCopy);
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
    return nullptr;
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

World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, World::WhichRobots whichRobots) {
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
            robotsCopy.insert(robotsCopy.end(), worldData.us.begin(), worldData.us.end());
            robotsCopy.insert(robotsCopy.end(), worldData.them.begin(), worldData.them.end());
            break;
        }
    }
    return rtt::ai::world::World::RobotPtr();
}

} //world
} //ai
} //rtt