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
    WorldData wd = WorldData(message);

    for (auto &robot : wd.us) {
        robot.updateRobot(wd.ball);
    }

    for (auto &robot : wd.them) {
        robot.updateRobot(wd.ball);
    }

    Ball oldBall = worldData->ball;
    wd.ball.updateBallModel(oldBall, wd);

    history.addWorld(wd);
    worldData = std::make_shared<WorldData>(wd);
}

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
    for (auto &robot : worldData->us) {
        auto robotMsg = makeWorldRobotMsg(robot);
        message.us.push_back(robotMsg);
    }
    for (auto &robot : worldData->them) {
        auto robotMsg = makeWorldRobotMsg(robot);
        message.them.push_back(robotMsg);
    }
    message.ball = makeWorldBallMsg(worldData->ball);
    message.time = worldData->time;
    return message;
}

bool World::weHaveRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    return worldData == nullptr ? false : !worldData->us.empty();
}

void World::setWorld(const roboteam_msgs::World &message) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldMsg = message;
    World::worldData = std::make_shared<WorldData>(WorldData(message));

}

void World::setWorldData(WorldDataPtr &worldDataPtr) {
    std::lock_guard<std::mutex> lockMsg(worldMsgMutex);
    std::lock_guard<std::mutex> lockyLock(worldMutex);

    World::worldData.reset();
    World::worldData = worldDataPtr;

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
        worldCopy = *worldData;
    }
    return worldCopy;
}

const World::BallPtr World::getBall() {
    Ball ballCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballCopy = worldData->ball;
    }
    if (ballCopy.exists != 0)
        return std::make_shared<Ball>(ballCopy);

    return BallPtr(nullptr);

}

const World::RobotPtr World::getRobotForId(int id, bool ourTeam) {
    WorldDataPtr worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    const std::vector<Robot> &robots = ourTeam ? worldCopy->us : worldCopy->them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return std::make_shared<Robot>(bot);
        }
    }
    return RobotPtr(nullptr);
}

const std::vector<World::RobotPtr> World::getAllRobots() {
    WorldDataPtr worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    std::vector<RobotPtr> allRobots;
    for (auto &robot : worldCopy->us)
        allRobots.push_back(std::make_shared<Robot>(robot));
    for (auto &robot : worldCopy->them)
        allRobots.push_back(std::make_shared<Robot>(robot));
    return allRobots;
}

const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> robots) {

    if (robots.empty())
        return RobotPtr(nullptr);

    Robot* closestRobot = &robots[0];
    for (auto &robot : robots) {
        if ((robot.pos - point).length() < (closestRobot->pos - point).length()) {
            closestRobot = &robot;
        }
    }
    return std::make_shared<Robot>(*closestRobot);
}

const World::RobotPtr World::getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots) {
    std::vector<Robot> robotsCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);

        switch (whichRobots) {
        case OUR_ROBOTS:
            robotsCopy = worldData->us;
            break;
        case THEIR_ROBOTS:
            robotsCopy = worldData->them;
            break;
        case ALL_ROBOTS:
        default:
            robotsCopy = worldData->us;
            robotsCopy.insert(robotsCopy.end(), worldData->them.begin(), worldData->them.end());
            break;
        }
    }

    getRobotClosestToPoint(point, robotsCopy);
}

const World::RobotPtr World::getRobotClosestToRobot(const RobotPtr &robot, WhichRobots whichRobots) {
    return getRobotClosestToPoint(robot->pos, whichRobots);
}

const World::RobotPtr World::getRobotClosestToBall(WhichRobots whichRobots) {
    Vector2 ballPos;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        ballPos = worldData->ball.pos;
    }

    return getRobotClosestToPoint(ballPos, whichRobots);
}

bool World::robotHasBall(int id, bool ourTeam, double maxDist) {
    return ourTeam ? ourRobotHasBall(id, maxDist) : theirRobotHasBall(id, maxDist);
}

bool World::ourRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, true);
    return robot->hasBall(maxDist);
}

bool World::theirRobotHasBall(int id, double maxDist) {
    auto robot = getRobotForId(id, false);
    return robot->hasBall(maxDist);
}

const World::RobotPtr World::whichRobotHasBall() {
    WorldDataPtr worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = worldData;
    }

    for (auto robot : worldCopy->us) {
        if (robot.hasBall()) std::make_shared<Robot>(robot);
    }
    for (auto robot : worldCopy->us) {
        if (robot.hasBall()) std::make_shared<Robot>(robot);
    }
    return RobotPtr(nullptr);
}

const WorldData World::getFutureWorld(double time) {
    WorldData worldCopy;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        worldCopy = *worldData;

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
        ballCopy = worldData->ball;
    }
    futureWorld.updateFutureBall(ballCopy, time);
    return std::make_shared<Ball>(ballCopy);
}


} //world
} //ai
} //rtt