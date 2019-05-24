#include "World.h"
#include "FutureWorld.h"
#include "BallPossession.h"

namespace rtt {
namespace ai {
namespace world {

World worldObj;
World* world = &worldObj;

void World::updateWorld(const roboteam_msgs::World &message) {
    worldNumber ++;

    BallPtr oldBall = nullptr;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        // create a worldData if there is none
        if (! worldDataPtr) {
            std::cout << "Creating first world" << std::endl;
            auto worldData = WorldData(message);
            worldDataPtr = std::make_shared<WorldData>(worldData);
        }
        if (worldDataPtr->ball) oldBall = std::make_shared<Ball>(*worldDataPtr->ball);
    }

    // update ballmodel, dribbling, position if not visible etc.
    auto tempWorldData = WorldData(message);
    if (oldBall) tempWorldData.ball->updateBall(oldBall, tempWorldData);

    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldDataPtr->time = message.time;
        updateBallFromData(tempWorldData.ball);
        updateRobotsFromData(Robot::us, message.us, worldDataPtr->us, worldDataPtr->ball, worldNumber);
        updateRobotsFromData(Robot::them, message.them, worldDataPtr->them, worldDataPtr->ball, worldNumber);

        // add the worlddata to the history
        WorldData worldDataCopyForHistory = WorldData(worldDataPtr);
        history.addWorld(worldDataCopyForHistory);
    }

    ballPossessionPtr->update();
}

void World::updateRobotsFromData(Robot::Team team, const std::vector<roboteam_msgs::WorldRobot> &robotsFromMsg,
        std::vector<RobotPtr> &robots, const BallPtr &ball, unsigned long newWorldNumber) const {
    for (auto &robotMsg : robotsFromMsg) {

        // find robots that are both in the vector and in the message
        bool robotFound = false;
        for (auto &robot : robots) {
            if (robot->id == robotMsg.id) {
                robotFound = true;
                if (robot) {
                    // if the robot already exists it should be updated
                    robot->updateRobot(robotMsg, ball, newWorldNumber);
                }
                break;
            }
        }
        // if no robot exists in world we create a new one
        if (!robotFound) {
            RobotPtr newRobot = std::make_shared<Robot>(Robot(robotMsg, team, worldNumber));
            newRobot->updateRobot(robotMsg, ball, worldNumber);

            // std::cout << "RobotPtr " << newRobot.id << " added to world" << std::endl;
            robots.emplace_back(newRobot);
        }
    }

    // check if some robots don't have new data. In that case remove them
    robots.erase(std::remove_if(robots.begin(), robots.end(), [=](RobotPtr robot) {
      return robot->getLastUpdatedWorldNumber() < worldNumber;
    }), robots.end());
}

bool World::weHaveRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return worldDataPtr && ! worldDataPtr->us.empty() && Ball::exists;
}

const WorldData World::getWorld() {
    return history.getPreviousWorld(0);
}

const World::BallPtr World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);

    if (world::Ball::exists) {
        return worldDataPtr->ball;
    }

    std::cerr << "BALL DOES NOT EXIST!!! (exists == 0 ??? )" << std::endl;
    return nullptr;
}

const World::RobotPtr World::getRobotForId(int id, bool ourTeam) {
    const std::vector<RobotPtr> robots = ourTeam ? getUs() : getThem();
    for (const auto &robot : robots) {
        if (robot->id == id) {
            return robot;
        }
    }
    return nullptr;
}

const std::vector<World::RobotPtr> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) {
        return {nullptr};
    }
    std::vector<RobotPtr> allRobots;
    allRobots.insert(allRobots.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
    allRobots.insert(allRobots.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
    return allRobots;
}

const std::vector<World::RobotPtr> World::getUs() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) return {nullptr};
    return worldDataPtr->us;
}

const std::vector<World::RobotPtr> World::getThem() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) return {nullptr};
    return worldDataPtr->them;
}

const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotPtr> &robots) {

    if (robots.empty()) return {nullptr};

    unsigned int bestIndex = 0;
    double closestDistance = 9e9;
    double distanceToCheck;
    for (unsigned int i = 0; i < robots.size(); i ++) {
        distanceToCheck = (robots[i]->pos - point).length();
        if (distanceToCheck < closestDistance) {
            closestDistance = distanceToCheck;
            bestIndex = i;
        }
    }

    return robots[bestIndex];
}

const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots) {
    std::vector<RobotPtr> robotsCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
            return {nullptr};
        }
        switch (whichRobots) {
        case OUR_ROBOTS:robotsCopy = worldDataPtr->us;
            break;
        case THEIR_ROBOTS:robotsCopy = worldDataPtr->them;
            break;
        case ALL_ROBOTS:
        default:robotsCopy.insert(robotsCopy.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
            robotsCopy.insert(robotsCopy.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
            break;
        }
    }

    return getRobotClosestToPoint(point, robotsCopy);
}

const World::RobotPtr World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
            return {nullptr};
        }
        ballPos = worldDataPtr->ball->pos;
    }

    return getRobotClosestToPoint(ballPos, whichRobots);
}


const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, std::vector<int> robotIds, bool ourTeam) {

    RobotPtr closestBot;
    double maxDist = 9e9;
    for (auto const &id : robotIds) {
        auto robot = getRobotForId(id, ourTeam);
        if (! robot) continue;
        auto dist = robot->pos.dist(point);
        if (dist < maxDist) {
            maxDist = dist;
            closestBot = robot;
        }
    }
    return closestBot;
}

bool World::robotHasBall(int id, bool ourTeam, double maxDist) {
    return ourTeam ? ourRobotHasBall(id, maxDist) : theirRobotHasBall(id, maxDist);
}

bool World::ourRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, true);
    if (! robot)
        return false;

    return robot->hasBall(maxDist);
}

bool World::theirRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, false);
    if (! robot)
        return false;

    return robot->hasBall(maxDist);
}

const World::RobotPtr World::whichRobotHasBall(WhichRobots whichRobots) {
    // checks for all robots which robot has the ball AND is closest to the ball
    std::vector<RobotPtr> allRobots;
    switch (whichRobots) {
    default: allRobots = getAllRobots();
        break;
    case OUR_ROBOTS: allRobots = getUs();
        break;
    case THEIR_ROBOTS: allRobots = getThem();
        break;
    case ALL_ROBOTS: allRobots = getAllRobots();
        break;
    }
    if (allRobots.empty()) {
        return nullptr;
    }

    double bestDistance = 9e9;
    RobotPtr bestRobot = nullptr;
    for (auto &robot : allRobots) {
        if (robot->hasBall()) {
            if (robot->getDistanceToBall() < bestDistance) {
                bestRobot = robot;
            }
        }
    }
    if (bestRobot) return getRobotForId(bestRobot->id, bestRobot->team == Robot::Team::us);

    return nullptr;
}

const WorldData World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
            return {};
        }
        worldCopy = WorldData(worldDataPtr);

    }

    futureWorld.updateFutureWorld(worldCopy, time);
    return worldCopy;
}

const World::RobotPtr World::getFutureRobot(int id, bool ourTeam, double time) {
    RobotPtr robotPtr = getRobotForId(id, ourTeam);
    return getFutureRobot(robotPtr, time);
}

const World::RobotPtr World::getFutureRobot(const RobotPtr &robot, double time) {
    if (!robot) return nullptr;
    Robot futureRobot = *robot;
    futureWorld.updateFutureRobot(futureRobot, time);
    return std::make_shared<Robot>(futureRobot);
}

const World::BallPtr World::getFutureBall(double time) {
    Ball ballCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
            return {nullptr};
        }
        ballCopy = (*worldDataPtr->ball);
    }
    futureWorld.updateFutureBall(ballCopy, time);
    return std::make_shared<Ball>(ballCopy);
}

const WorldData World::getPreviousWorld() {
    return history.getPreviousWorld();
}

double World::getTimeDifference() {
    return worldDataPtr->time - getPreviousWorld().time;
}

const std::vector<World::RobotPtr> World::getRobotsForIds(std::vector<int> ids, bool ourTeam) {
    std::vector<World::RobotPtr> robots;
    for (auto const &id : ids) {
        auto robot = getRobotForId(id, ourTeam);
        if (robot) {
            robots.push_back(robot);
        }
    }
    return robots;
}

double World::getTime() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return worldDataPtr->time;
}

void World::updateBallFromData(const World::BallPtr &ball) {
    if (ball) worldDataPtr->ball = ball;
}

} //world
} //ai
} //rtt