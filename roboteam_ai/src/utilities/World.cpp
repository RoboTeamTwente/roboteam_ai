#include <utility>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
std::vector<std::pair<roboteam_msgs::World, double>> World::futureWorlds;

bool World::didReceiveFirstWorld = false;
std::map<int, double> World::OurBotsBall;
std::map<int, double> World::TheirBotsBall;

std::mutex World::worldMutex;

/// return the world message
const roboteam_msgs::World &World::get_world() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return World::world;
}

/// set a world message
/// if there is an 'us' vector, it sets didReceiveWorld to true
void World::set_world(roboteam_msgs::World _world) {
    std::lock_guard<std::mutex> lock(worldMutex);
    futureWorlds = {{},{}};

    if (! _world.us.empty()) {
        didReceiveFirstWorld = true;
    }
    if (!_world.ball.visible){
        _world.ball=updateBallPosition(_world);
    }
    updateBallPossession(_world);
    world = _world;
}

/// Returns Robot for an ID and team (if it exists) 
std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotForId(unsigned int id, bool robotIsOurTeam) {
    roboteam_msgs::World _world;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        _world = world;
    }
    const std::vector<roboteam_msgs::WorldRobot> &robots = robotIsOurTeam ? _world.us : _world.them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return std::make_shared<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return nullptr;
}

/// returns robots for multiple ids and a team (if they are exist, otherwise the non-existent robots are nullptr)
std::vector<roboteam_msgs::WorldRobot> World::getRobotsForId(std::set<unsigned int> ids, bool robotsAreOurTeam) {
    std::vector<roboteam_msgs::WorldRobot> robots;
    for (const unsigned int &id : ids) {
        auto robot = getRobotForId(id, robotsAreOurTeam);
        if (robot) {
            robots.push_back(*robot);
        }
    }
    return robots;
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point, const int &myID, const float &t) {

    std::shared_ptr<roboteam_msgs::WorldRobot> closestRobot = nullptr;
    double distance = 99999999;

    for (auto &bot : robots) {
        if (static_cast<int>(bot.id) != myID) {
            Vector2 botPosAtT = (Vector2)bot.pos + (Vector2)bot.vel * t;
            double botDist = (botPosAtT - point).length();
            if (botDist < distance) {
                closestRobot = std::make_shared<roboteam_msgs::WorldRobot>(bot);
                distance = botDist;
            }
        }
    }
    return closestRobot;
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point, const int &myID) {

    const float t = 0;
    return getRobotClosestToPoint(std::move(robots), point, myID, t);
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point) {

    const int myID = -1;
    const float t = 0;
    return getRobotClosestToPoint(std::move(robots), point, myID, t);
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point, const float &t) {

    const int myID = -1;
    return getRobotClosestToPoint(std::move(robots), point, myID, t);
}


std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(const Vector2 &point, const int &myID, const float &t) {

    auto closestUs = getRobotClosestToPoint(World::get_world().us, point, myID, t);
    auto closestThem = getRobotClosestToPoint(World::get_world().them, point, t);
    double lengthUs = closestUs != nullptr ? (point - closestUs->pos).length() : 9999.0;
    double lengthThem = closestThem != nullptr ? (point - closestThem->pos).length() : 9999.0;
    return lengthUs < lengthThem ? closestUs : closestThem;
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(const Vector2 &point, const float &t) {

    const int myID = -1;
    return getRobotClosestToPoint(point, myID, t);
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(const Vector2 &point, const int &myID) {

    const float t = 0;
    return getRobotClosestToPoint(point, myID, t);
}

/// returns the ball msg
std::shared_ptr<roboteam_msgs::WorldBall> World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return std::make_shared<roboteam_msgs::WorldBall>(world.ball);
}


double World::findBallDist(roboteam_msgs::WorldRobot &bot, roboteam_msgs::WorldBall &ball) {
    Vector2 robotPos = Vector2(bot.pos);
    double robotOrientation = bot.angle;
    Vector2 ballPos = ball.pos;
    Vector2 dribbleLeft = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation - Constants::DRIBBLER_ANGLE_OFFSET());
    Vector2 dribbleRight = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation + Constants::DRIBBLER_ANGLE_OFFSET());
    // if the ball is very close (or on top of the robot?)
    if (control::ControlUtils::pointInTriangle(ballPos, robotPos, dribbleLeft, dribbleRight)) {
        return 0.0;
    }
    else if (control::ControlUtils::pointInRectangle(ballPos, dribbleLeft, dribbleRight,
            dribbleRight + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation),
            dribbleLeft + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation))) {
        return control::ControlUtils::distanceToLine(ballPos, dribbleLeft, dribbleRight);
    }
    // default return
    return - 1.0;

}
void World::updateBallPossession(roboteam_msgs::World &_world) {
    OurBotsBall.clear();
    TheirBotsBall.clear();
    // calculate if a bot is in dribbling range and if so what range it is at.
    for (auto ourBot :_world.us) {
        double dist = findBallDist(ourBot, _world.ball);
        if (dist != - 1.0) {
            OurBotsBall[ourBot.id] = dist;
        }
    }
    for (auto theirBot: _world.them) {
        double dist = findBallDist(theirBot, _world.ball);
        if (dist != - 1.0) {
            TheirBotsBall[theirBot.id] = dist;
        }
    }
}
// uses MAX_BALL_RANGE as default max Dist
bool World::botHasBall(int id, bool ourTeam, double maxDistToBall) {
    if (ourTeam){
        return ourBotHasBall(id,maxDistToBall);
    }
    else{
        return theirBotHasBall(id,maxDistToBall);
    }
}
bool World::ourBotHasBall(int id, double maxDistToBall){
    std::lock_guard<std::mutex> lock(worldMutex);
    if (OurBotsBall.find(id)!=OurBotsBall.end()){
        if (OurBotsBall[id]<=maxDistToBall){
            return true;
        }
    }
    return false;
}
bool World::theirBotHasBall(int id, double maxDistToBall){
    std::lock_guard<std::mutex> lock(worldMutex);
    if (TheirBotsBall.find(id)!=TheirBotsBall.end()){
        if (TheirBotsBall[id]<=maxDistToBall){
            return true;
        }
    }
    return false;
}
// picks the bot that has the ball and is closest to it.
int World::whichBotHasBall(bool ourTeam) {
    std::lock_guard<std::mutex> lock(worldMutex);
    double maxDist = 100;
    int bestId = - 1;
    if (ourTeam) {
        for (auto bot: OurBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
            }
        }
    }
    else {
        for (auto bot: TheirBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
            }
        }
    }
    return bestId;
}
/// returns a message of the world where every position has been linearly extrapolated w.r.t current world
roboteam_msgs::World World::futureWorld(double time, double maxTimeOffset) {
    //std::cout << std::endl << "futureWorlds: " << futureWorlds.size() << std::endl;
//    for (auto futureWorld : futureWorlds) {
//        if (abs(time - futureWorld.second) < maxTimeOffset)
//            return futureWorld.first;
//    }

    roboteam_msgs::World currentWorld;
    {
    std::lock_guard<std::mutex> lock(worldMutex);
    currentWorld = world;
    }

    roboteam_msgs::World futureWorld;
    for(auto bot :currentWorld.us){
        bot.pos=Vector2(bot.pos)+Vector2(bot.vel)*time;
        futureWorld.us.push_back(bot);
    }
    for(auto bot : currentWorld.them){
        bot.pos=Vector2(bot.pos)+Vector2(bot.vel)*time;
        futureWorld.them.push_back(bot);
    }
    futureWorld.ball=currentWorld.ball;
    futureWorld.ball.pos=Vector2(currentWorld.ball.pos)+Vector2(currentWorld.ball.vel)*time;

    //futureWorlds.emplace_back(futureWorld, time);
    return futureWorld;
}

/// returns all the robots in the field, both us and them.
roboteam_msgs::WorldBall World::updateBallPosition(roboteam_msgs::World _world) {
    roboteam_msgs::WorldBall newBall=_world.ball;
    if (_world.ball.visible){
        return newBall;
    }
    // we set the ball velocity to 0
    newBall.vel.x=0;
    newBall.vel.y=0;
    // if the ball was dribbled in the previous world_state we set its position to be in front of the robot that was dribbling it
    if (!OurBotsBall.empty()||!TheirBotsBall.empty()){ // check if it was dribbled in last world state
        double maxDist=100;
        int bestId=-1;
        bool ourTeam=true;
        for (auto bot: OurBotsBall ) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = true;
            }
        }
        for (auto bot: TheirBotsBall ) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = false;
            }
        }
        if (bestId==-1){
            ROS_ERROR("Could not find a proper bot that possesses ball!");
            return newBall;
        }
        // I can't use getRobotForId because of deadlocking and because i need to use the world in the packet
        roboteam_msgs::WorldRobot robot;
        const std::vector<roboteam_msgs::WorldRobot> &robots = ourTeam ? _world.us: _world.them;
        for (const auto &bot : robots) {
            if (bot.id == bestId) {
                robot=bot;
                break;
            }
        }
        // put the ball in front of the centre robot that is dribbling it.
        Vector2 ballPos=Vector2(robot.pos)+Vector2(Constants::CENTRE_TO_FRONT()+Constants::BALL_RADIUS(),0).rotate(robot.angle);
        newBall.pos=ballPos;
    }
    // else (not visible but not dribbled), we put it at its previous position
    else{
        newBall.pos = world.ball.pos;
    }
    return newBall;
}


std::pair<int, bool> World::getRobotClosestToBall() {
    auto closestUs = World::getRobotClosestToPoint(World::get_world().us, World::getBall()->pos);
    auto closestThem = World::getRobotClosestToPoint(World::get_world().them, World::getBall()->pos);

    auto distanceToBallUs = (Vector2(closestUs->pos).dist(Vector2(World::getBall()->pos)));
    auto distanceToBallThem = (Vector2(closestThem->pos).dist(Vector2(World::getBall()->pos)));

    roboteam_msgs::WorldRobot closestRobot;
    bool weAreCloser;

    if (distanceToBallUs < distanceToBallThem) {
        closestRobot = * closestUs;
        weAreCloser = true;
    } else {
        closestRobot = * closestThem;
        weAreCloser = false;
    }

    return std::make_pair(closestRobot.id, weAreCloser);
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToBall(bool isOurTeam) {
    return World::getRobotClosestToPoint(isOurTeam ? World::get_world().us : World::get_world().them, World::getBall()->pos);
}

/// returns all the robots in the field, both us and them.
std::vector<roboteam_msgs::WorldRobot> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    std::vector<roboteam_msgs::WorldRobot> allRobots;
    allRobots.insert(allRobots.end(), world.us.begin(), world.us.end());
    allRobots.insert(allRobots.end(), world.them.begin(), world.them.end());
    return allRobots;
}

} // ai
} // rtt