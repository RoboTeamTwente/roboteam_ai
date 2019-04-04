//
// Created by thijs on 19-3-19.
//

#include "World.h"
#include "FutureWorld.h"

namespace rtt {
namespace ai {
namespace world {


World worldObj;
World* world = &worldObj;

void World::updateWorld(const roboteam_msgs::World &message) {
    WorldData newWorldData = WorldData(message);
    Ball oldBall;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        if (! worldDataPtr) {
            worldData = newWorldData;
            worldDataPtr = std::make_shared<WorldData>(worldData);
        }
        oldBall = worldDataPtr->ball;
    }
    newWorldData.ball.updateBall(oldBall, newWorldData);
    for (auto &robot : newWorldData.us) {
        robot.updateRobot(newWorldData.ball);
    }
    for (auto &robot : newWorldData.them) {
        robot.updateRobot(newWorldData.ball);
    }
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        worldData = newWorldData;
        history.addWorld(worldData);
        worldDataPtr = std::make_shared<WorldData>(World::worldData);
    }
}
//TODO:: MAKE THIS IN ROBOT.CPP
roboteam_msgs::WorldRobot World::makeWorldRobotMsg(const Robot& robot) {
    roboteam_msgs::WorldRobot robotMsg;
    robotMsg.angle = static_cast<float>(robot.angle);
    robotMsg.w = static_cast<float>(robot.angularVelocity);
    robotMsg.pos = robot.pos;
    robotMsg.vel = robot.vel;
    robotMsg.id = static_cast<unsigned int>(robot.id);
    return robotMsg;
}

roboteam_msgs::WorldBall World::makeWorldBallMsg(const Ball& ball) {
    roboteam_msgs::WorldBall ballMsg;
    ballMsg.existence = static_cast<unsigned int>(ball.exists);
    ballMsg.visible = static_cast<unsigned char>(ball.visible);
    ballMsg.pos = ball.pos;
    ballMsg.vel = ball.vel;
    return ballMsg;
}

roboteam_msgs::World World::makeWorldMsg() {
    roboteam_msgs::World message;
    for (auto &robot : worldDataPtr->us) {
        auto robotMsg = makeWorldRobotMsg(robot);
        message.us.push_back(robotMsg);
    }
    for (auto &robot : worldDataPtr->them) {
        auto robotMsg = makeWorldRobotMsg(robot);
        message.them.push_back(robotMsg);
    }
    message.ball = makeWorldBallMsg(worldDataPtr->ball);
    message.time = worldDataPtr->time;
    return message;
}

bool World::weHaveRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    return worldDataPtr == nullptr ? false : !worldDataPtr->us.empty();
}

void World::setWorld(const roboteam_msgs::World &message) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldMsg = message;
    World::worldDataPtr = std::make_shared<WorldData>(WorldData(message));

}

void World::setWorldData(WorldDataPtr &worldDataPtr) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldDataPtr.reset();
    World::worldDataPtr = worldDataPtr;

    World::worldMsg = makeWorldMsg();
}

const roboteam_msgs::World &World::getWorldMsg() {
    std::lock_guard<std::mutex> lock(worldMsgMutex);

    return worldMsg;
}

const roboteam_msgs::WorldBall &World::getBallMsg() {
    std::lock_guard<std::mutex> lock(worldMsgMutex);

    return worldMsg.ball;
}

const WorldData World::getWorld() {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (worldDataPtr) {
            worldCopy = *worldDataPtr;
        }
    }
    return worldCopy;
}

const World::BallPtr World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);

    if (worldDataPtr && worldDataPtr->ball.exists)
        return std::make_shared<Ball>(worldDataPtr->ball);

    std::cerr << "BALL DOES NOT EXIST!!! (exists == 0 ??? )" << std::endl;
    return BallPtr(nullptr);
}

const World::RobotPtr World::getRobotForId(int id, bool ourTeam) {
    WorldDataPtr worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldDataPtr;
    }

    const std::vector<Robot> &robots = ourTeam ? worldCopy->us : worldCopy->them;
    for (const auto &robot : robots) {
        if (robot.id == id) {
            return std::make_shared<Robot>(robot);
        }
    }
    return RobotPtr(nullptr);
}

const std::vector<Robot> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);
    std::vector<Robot> allRobots;
    allRobots.insert(allRobots.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
    allRobots.insert(allRobots.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
    return allRobots;
}

const std::vector<Robot> World::getUs() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return worldDataPtr->us;
}

const std::vector<Robot> World::getThem() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return worldDataPtr->them;
}

Robot World::getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> robots) {

    if (robots.empty())
        return {};

    unsigned int bestIndex = 0;
    double closestDistance = 9999999;
    double distanceToCheck;
    for (unsigned int i = 0; i < robots.size(); i++) {
        distanceToCheck = (robots[i].pos - point).length();
        if (distanceToCheck < closestDistance) {
            closestDistance = distanceToCheck;
            bestIndex = i;
        }
    }

    return robots[bestIndex];
}

Robot World::getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots) {
    std::vector<Robot> robotsCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        switch (whichRobots) {
        case OUR_ROBOTS:
            robotsCopy = worldDataPtr->us;
            break;
        case THEIR_ROBOTS:
            robotsCopy = worldDataPtr->them;
            break;
        case ALL_ROBOTS:
        default:
            robotsCopy.insert(robotsCopy.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
            robotsCopy.insert(robotsCopy.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
            break;
        }
    }

    return getRobotClosestToPoint(point, robotsCopy);
}

Robot World::getRobotClosestToRobot(const RobotPtr &robot, WhichRobots whichRobots) {
    auto allRobots = getAllRobots();
    if (!robot || allRobots.empty())
        return {};

    auto robotPos = robot->pos;
    unsigned int bestIndex = 0;
    double closestDistance = 9999999;
    double distanceToCheck;

    for (unsigned int i = 0; i < allRobots.size(); i++) {
        if (allRobots[i].id == robot->id && allRobots[i].team == robot->team) {
            distanceToCheck = (allRobots[i].pos - robotPos).length();
            if (distanceToCheck < closestDistance) {
                closestDistance = distanceToCheck;
                bestIndex = i;
            }
        }
    }
    return allRobots[bestIndex];
}

Robot World::getRobotClosestToRobot(int id, bool ourTeam, WhichRobots whichRobots) {
    RobotPtr robot = getRobotForId(id, ourTeam);
    if (!robot)
        return {};

    return getRobotClosestToRobot(robot, whichRobots);
}

Robot World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballPos = worldDataPtr->ball.pos;
    }

    return getRobotClosestToPoint(ballPos, whichRobots);
}

bool World::robotHasBall(int id, bool ourTeam, double maxDist) {
    return ourTeam ? ourRobotHasBall(id, maxDist) : theirRobotHasBall(id, maxDist);
}

bool World::ourRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, true);
    if (!robot)
        return false;

    return robot->hasBall(maxDist);
}

bool World::theirRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, false);
    if (!robot)
        return false;

    return robot->hasBall(maxDist);
}

const World::RobotPtr World::whichRobotHasBall() {
    auto allRobots = getAllRobots();
    double bestDistance = 999999;
    Robot bestRobot = {};
    for (auto robot : allRobots) {
        if (robot.hasBall()) {
            if (robot.getDistanceToBall() < bestDistance) {
                bestRobot = robot;
            }
        }
    }
    if (bestRobot.id != -1)
        return std::make_shared<Robot>(bestRobot);

    return RobotPtr(nullptr);
}

const WorldData World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = *worldDataPtr;

    }

    futureWorld.updateFutureWorld(worldCopy, time);
    return worldCopy;
}

const World::RobotPtr World::getFutureRobot(int id, bool ourTeam, double time) {
    RobotPtr robotCopy = getRobotForId(id, ourTeam);
    if (!robotCopy) {
        return RobotPtr(nullptr);
    }

    Robot robot = *robotCopy;
    futureWorld.updateFutureRobot(robot, time);
    return std::make_shared<Robot>(robot);
}

const World::BallPtr World::getFutureBall(double time) {
    Ball ballCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballCopy = worldDataPtr->ball;
    }
    futureWorld.updateFutureBall(ballCopy, time);
    return std::make_shared<Ball>(ballCopy);
}


} //world
} //ai
} //rtt