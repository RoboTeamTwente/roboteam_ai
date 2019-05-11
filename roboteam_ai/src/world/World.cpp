#include "World.h"
#include "FutureWorld.h"
#include "BallPossession.h"

namespace rtt {
namespace ai {
namespace world {

World worldObj;
World* world = &worldObj;

void World::updateWorld(const roboteam_msgs::World &message) {

    worldNumber++;

    // check if there is a previous worldstate
    // otherwise make the worlddata item
    {
        std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) {
        worldData = WorldData(message);
        worldDataPtr = std::make_shared<WorldData>(worldData);
    }

    worldDataPtr->ball.pos = message.ball.pos;
    updateRobotsFromData(message.us, worldDataPtr->us, worldDataPtr->ball, worldNumber);
    updateRobotsFromData(message.them, worldDataPtr->them, worldDataPtr->ball, worldNumber);

    }
    bpTracker->update();
}

    void World::updateRobotsFromData(const std::vector<roboteam_msgs::WorldRobot> &robotsFromMsg, std::vector<Robot> &robots, const Ball &ball, unsigned long worldNumber) const {
        for (auto robotMsg : robotsFromMsg) {

            // find robots that are both in the vector and in the message
            auto robot = find_if(robots.begin(), robots.end(), [&robotMsg](const Robot &obj) {
                return obj.id == robotMsg.id;
            });

            bool robotExistsInWorld = robot != robots.end();
            if (robotExistsInWorld) {
                // if the robot already exists it should be updated
                robot->updateRobot(robotMsg, ball, worldNumber);
            } else {
                // if no robot exists in world we create a new one
                Robot newRobot(robotMsg, Robot::us);
                robots.push_back(newRobot);
            }
        }

        // check if some robots don't have new data. In that case remove them
        robots.erase(std::remove_if(robots.begin(), robots.end(), [=](Robot robot) {
            return robot.getLastUpdatedWorldNumber() < worldNumber;
        }), robots.end());
    }

    const roboteam_msgs::WorldRobot World::makeWorldRobotMsg(const Robot &robot) {
    return robot.toMessage();
}

const roboteam_msgs::WorldBall World::makeWorldBallMsg(const Ball &ball) {
    return ball.toMessage();
}

const roboteam_msgs::World World::makeWorldMsg() {
    if (! worldDataPtr)
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
    return worldDataPtr && ! worldDataPtr->us.empty();
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
        if (! worldDataPtr) {
            return RobotPtr(nullptr);
        }
        worldCopy = worldDataPtr;

    }
    const std::vector<Robot> robots = ourTeam ? worldCopy->us : worldCopy->them;
    for (const auto &robot : robots) {
        if (robot.id == id) {
            return std::make_shared<Robot>(robot);
        }
    }
    return RobotPtr(nullptr);
}

const std::vector<Robot> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) {
        return {};
    }
    std::vector<Robot> allRobots;
    allRobots.insert(allRobots.end(), worldDataPtr->us.begin(), worldDataPtr->us.end());
    allRobots.insert(allRobots.end(), worldDataPtr->them.begin(), worldDataPtr->them.end());
    return allRobots;
}

const std::vector<Robot> World::getUs() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) return {};
    return worldDataPtr->us;
}

const std::vector<Robot> World::getThem() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldDataPtr) return {};
    return worldDataPtr->them;
}

Robot World::getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> robots) {

    if (robots.empty()) return {};

    unsigned int bestIndex = 0;
    double closestDistance = 9e9;
    double distanceToCheck;
    for (unsigned int i = 0; i < robots.size(); i ++) {
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
        if (! worldDataPtr) {
            return {};
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

Robot World::getRobotClosestToRobot(const RobotPtr &robot, WhichRobots whichRobots) {
    auto allRobots = getAllRobots();
    if (! robot || allRobots.empty())
        return {};

    auto robotPos = robot->pos;
    unsigned int bestIndex = 0;
    double closestDistance = 9e9;
    double distanceToCheck;

    for (unsigned int i = 0; i < allRobots.size(); i ++) {
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
    if (! robot) return {};
    return getRobotClosestToRobot(robot, whichRobots);
}

Robot World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
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
    std::vector<Robot> allRobots;
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
    if (bestRobot.id != - 1)
        return std::make_shared<Robot>(bestRobot);

    return RobotPtr(nullptr);
}

const WorldData World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldDataPtr) {
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
    if (! robotPtr) {
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
        if (! worldDataPtr) {
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
        if (!robot) continue;
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