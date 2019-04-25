//
// Created by thijs on 19-3-19.
//

#include "World.h"
#include "FutureWorld.h"
#include "BallPossession.h"

namespace rtt {
namespace ai {
namespace world {


World worldObj;
World* world = &worldObj;

void World::updateWorld(const roboteam_msgs::World &message) {
    // check if there is a previous worldstate
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        if (! worldDataPtr) {
            worldData = WorldData(message);
            worldDataPtr = std::make_shared<WorldData>(worldData);
        }
    }

    // convert roboteam_msgs::World into WorldData
    WorldData newWorldData = WorldData(message);

    // get the old ball from world
    Ball oldBall;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        oldBall = worldDataPtr->ball;
    }

    // update ballmodel, dribbling, position if not visible etc.
    newWorldData.ball.updateBall(oldBall, newWorldData);

    // update distance to ball, if it has ball etc.
    for (auto &robot : newWorldData.us) {
        robot.updateRobot(newWorldData.ball);
    }
    for (auto &robot : newWorldData.them) {
        robot.updateRobot(newWorldData.ball);
    }

    // set the new world
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        worldData = newWorldData;
        history.addWorld(worldData);
        worldDataPtr = std::make_shared<WorldData>(World::worldData);
    }
    bpTracker->update();
}

const roboteam_msgs::WorldRobot World::makeWorldRobotMsg(const Robot& robot) {
    return robot.toMessage();
}

const roboteam_msgs::WorldBall World::makeWorldBallMsg(const Ball& ball) {
    return ball.toMessage();
}

const roboteam_msgs::World World::makeWorldMsg() {
    if (!worldDataPtr)
        return {};

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
    return worldDataPtr && !worldDataPtr->us.empty();
}

void World::setWorldData(WorldDataPtr &setWorldDataPtr) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldDataPtr.reset();
    World::worldDataPtr = setWorldDataPtr;

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

    if (world::Ball::exists) {
        return std::make_shared<Ball>(worldDataPtr->ball);
    }

    std::cerr << "BALL DOES NOT EXIST!!! (exists == 0 ??? )" << std::endl;
    return BallPtr(nullptr);
}

const World::RobotPtr World::getRobotForId(int id, bool ourTeam) {
    WorldDataPtr worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (!worldDataPtr) {
            return RobotPtr(nullptr);
        }
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
    if (!worldDataPtr) {
        return {};
    }
    std::vector<Robot> allRobots;
    allRobots.insert(allRobots.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
    allRobots.insert(allRobots.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
    return allRobots;
}

const std::vector<Robot> World::getUs() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (!worldDataPtr) return {};
    return worldDataPtr->us;
}

const std::vector<Robot> World::getThem() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (!worldDataPtr) return {};
    return worldDataPtr->them;
}

Robot World::getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> robots) {

    if (robots.empty()) return {};

    unsigned int bestIndex = 0;
    double closestDistance = 9e9;
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
        if (!worldDataPtr) {
            return {};
        }
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
    double closestDistance = 9e9;
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
    if (!robot) return {};
    return getRobotClosestToRobot(robot, whichRobots);
}

Robot World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (!worldDataPtr) {
            return {};
        }
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

const World::RobotPtr World::whichRobotHasBall(WhichRobots whichRobots) {
    // checks for all robots which robot has the ball AND is closest to the ball
    std::vector<Robot> allRobots;
    if (whichRobots==WhichRobots::OUR_ROBOTS){
        allRobots=getUs();
    }
    else if (whichRobots==WhichRobots::THEIR_ROBOTS){
        allRobots=getThem();
    }
    else{
        allRobots=getAllRobots();
    }
    if (allRobots.empty()) {
        return RobotPtr(nullptr);
    }

    double bestDistance = 9e9;
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
        if (!worldDataPtr) {
            return {};
        }
        worldCopy = *worldDataPtr;

    }

    futureWorld.updateFutureWorld(worldCopy, time);
    return worldCopy;
}

const World::RobotPtr World::getFutureRobot(int id, bool ourTeam, double time) {
    RobotPtr robotPtr = getRobotForId(id, ourTeam);
    return getFutureRobot(robotPtr, time);
}

const World::RobotPtr World::getFutureRobot(const RobotPtr &robotPtr, double time) {
    if (!robotPtr) {
        return RobotPtr(nullptr);
    }
    Robot robot = getFutureRobot(*robotPtr, time);
    return std::make_shared<Robot>(robot);
}

const Robot World::getFutureRobot(const Robot &robot, double time) {
    Robot futureRobot = robot;
    futureWorld.updateFutureRobot(futureRobot, time);
    return futureRobot;
}

const World::BallPtr World::getFutureBall(double time) {
    Ball ballCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (!worldDataPtr) {
            return {};
        }
        ballCopy = worldDataPtr->ball;
    }
    futureWorld.updateFutureBall(ballCopy, time);
    return std::make_shared<Ball>(ballCopy);
}

const WorldData World::getPreviousWorld() {
    return history.getPreviousWorld();
}

double World::timeDifference() {
    return worldDataPtr->time - getPreviousWorld().time;
}

const std::vector<world::Robot> World::getRobotsForIds(std::vector<int> ids, bool ourTeam) {
    std::vector<world::Robot> robots;
    for (auto const &id : ids) {
        auto robot = getRobotForId(id, ourTeam);
        if (robot) {
            robots.push_back(* robot);
        }
    }
    return robots;
}

Robot World::getRobotClosestToPoint(const Vector2 &point, std::vector<int> robotIds, bool ourTeam) {

    Robot closestBot;
    double maxDist = INT_MAX;
    for (auto const &id : robotIds) {
        auto robot = getRobotForId(id, ourTeam);
        auto dist = robot->pos.dist(point);
        if (dist < maxDist) {
            maxDist = dist;
            closestBot = * robot;
        }
    }
    return closestBot;
}

} //world
} //ai
} //rtt