#include "world_old/World.h"
#include "world_old/FutureWorld.h"
#include "world_old/BallPossession.h"
#include "world_old/History.h"

namespace rtt::ai::world {

World worldObj;
World* world = &worldObj;

void World::updateWorld(const proto::World &message) {
    worldNumber ++;

    BallPtr oldBall = nullptr;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        // create a worldData if there is none
        if (!worldData) {
            std::cout << "Creating first world" << std::endl;
            worldData = WorldData(message, *rtt::Settings::settings);

        }

        // copy the ball
        if (worldData->ball) {
            oldBall = std::make_shared<Ball>(*worldData->ball);
        }
    }

    // update ballmodel, dribbling, position if not visible etc.
    auto tempWorldData = WorldData(message, *rtt::Settings::settings);
    if (oldBall) {
        tempWorldData.ball->updateBall(oldBall, tempWorldData);
    }
    else {
        tempWorldData.ball->updateBall(tempWorldData.ball, tempWorldData);
    }

    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldData->ball = tempWorldData.ball;
        worldData->time = message.time();

        std::vector<proto::WorldRobot>usMsg;
        std::vector<proto::WorldRobot>themMsg;

        if (rtt::Settings::settings->isYellow()) {
            usMsg = std::vector<proto::WorldRobot>(message.yellow().begin(), message.yellow().end());
            themMsg = std::vector<proto::WorldRobot>(message.blue().begin(), message.blue().end());
        } else {
            usMsg = std::vector<proto::WorldRobot>(message.blue().begin(), message.blue().end());
            themMsg = std::vector<proto::WorldRobot>(message.yellow().begin(), message.yellow().end());
        }
        updateRobotsFromData(us, usMsg, worldData->us, worldData->ball, worldNumber);
        updateRobotsFromData(them, themMsg, worldData->them, worldData->ball, worldNumber);

        // add the worlddata to the history
        WorldData worldDataCopyForHistory = WorldData(*worldData);
        history->addWorld(worldDataCopyForHistory);
    }

    ballPossessionPtr->update();
}

void World::updateRobotsFromData(Team team, const std::vector<proto::WorldRobot> &robotsFromMsg,
        std::vector<RobotPtr> &robots, const BallPtr &ball, unsigned long newWorldNumber) const {
    for (auto robotMsg : robotsFromMsg) {

        // find robots that areor/ both in the vector and in the message
        bool robotFound = false;
        for (auto &robot : robots) {
            if (robot->id == robotMsg.id()) {
                robotFound = true;
                if (robot) {
                    // if the robot already exists it should be updated
                    robot->updateRobot(robotMsg, ball, newWorldNumber);
                }
                break;
            }
        }
        // if no robot exists in world we create a new one
        if (! robotFound) {
            RobotPtr newRobot = std::make_shared<Robot>(Robot(robotMsg, team, 3, 0, worldNumber));
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
    return worldData && !worldData->us.empty();
}

const WorldData World::getWorld() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return WorldData(*worldData);
}

World::BallPtr World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);

    if (world::Ball::exists) {
        return worldData->ball;
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
    if (! worldData) {
        return {};
    }
    std::vector<RobotPtr> allRobots;
    allRobots.insert(allRobots.end(), worldData->us.begin(), worldData->us.end());
    allRobots.insert(allRobots.end(), worldData->them.begin(), worldData->them.end());
    return allRobots;
}

const std::vector<World::RobotPtr> World::getUs() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (! worldData)
        return {};
    return worldData->us;
}

const std::vector<World::RobotPtr> World::getThem() {
    std::lock_guard<std::mutex> lock(worldMutex);
    if (!worldData) return {};
    return worldData->them;
}

const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotPtr> &robots) {

    if (robots.empty()) return {};

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
        if (! worldData) {
            return {nullptr};
        }
        switch (whichRobots) {
        case OUR_ROBOTS:robotsCopy = worldData->us;
            break;
        case THEIR_ROBOTS:robotsCopy = worldData->them;
            break;
        case ALL_ROBOTS:
        default:robotsCopy.insert(robotsCopy.end(), worldData->us.begin(), worldData->us.end());
            robotsCopy.insert(robotsCopy.end(), worldData->them.begin(), worldData->them.end());
            break;
        }
    }

    return getRobotClosestToPoint(point, robotsCopy);
}

const World::RobotPtr World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (! worldData) {
            return {};
        }
        ballPos = worldData->ball->getPos();
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
    if (bestRobot) return getRobotForId(bestRobot->id, bestRobot->team == Team::us);

    return nullptr;
}

const std::optional<WorldData> World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (!worldData) {
            return std::nullopt;
        }
        worldCopy = WorldData(worldData.value());
    }

    futureWorld->updateFutureWorld(worldCopy, time);
    return worldCopy;
}

const World::RobotPtr World::getFutureRobot(int id, bool ourTeam, double time) {
    RobotPtr robotPtr = getRobotForId(id, ourTeam);
    if (! robotPtr) return nullptr;
    return getFutureRobot(robotPtr, time);
}

const World::RobotPtr World::getFutureRobot(const RobotPtr &robot, double time) {
    if (! robot) return nullptr;
    auto futureRobot = std::make_shared<world::Robot>(Robot(*robot));
    futureWorld->updateFutureRobot(futureRobot, time);
    return futureRobot;
}

const World::BallPtr World::getFutureBall(double time) {
    BallPtr futureBall;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        if (!worldData || ! worldData->ball) {
            return nullptr;
        }
        futureBall = std::make_shared<world::Ball>(Ball(*worldData->ball));
    }
    futureWorld->updateFutureBall(futureBall, time);
    return futureBall;
}

const WorldData World::getPreviousWorld() {
    return history->getPreviousWorld();
}

double World::getTimeDifference() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return worldData->time - getPreviousWorld().time;
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
    return worldData->time;
}

World::World() noexcept
    : futureWorld{ std::make_unique<FutureWorld>() }, history{ std::make_unique<History>() }
    {}

} //rtt