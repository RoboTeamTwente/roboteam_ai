#include <utility>

#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
std::vector<std::pair<roboteam_msgs::World, double>> World::futureWorlds;

bool World::didReceiveFirstWorld = false;
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
    if (! _world.ball.visible) {
        _world.ball = world.ball;
        _world.ball.visible = false;
    }
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

/// returns boolean if a robot for a given ID has the ball
bool World::robotHasBall(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball, double frontDist) {
    return World::robotHasBall(bot.pos, bot.angle, ball.pos, frontDist);
}

/// returns true if a robot at a given position and orientation has the ball
bool World::robotHasBall(Vector2 robotPos, double robotOrientation, Vector2 ballPos, double frontDist) {
    Vector2 dribbleLeft =
            robotPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation - Constants::DRIBBLER_ANGLE_OFFSET());
    Vector2 dribbleRight =
            robotPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation + Constants::DRIBBLER_ANGLE_OFFSET());

    if (control::ControlUtils::pointInTriangle(ballPos, robotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
        // else check the rectangle in front of the robot.
    else
        return control::ControlUtils::pointInRectangle(ballPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(frontDist, 0).rotate(robotOrientation),
                dribbleLeft + Vector2(frontDist, 0).rotate(robotOrientation));
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
std::vector<roboteam_msgs::WorldRobot> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    std::vector<roboteam_msgs::WorldRobot> allRobots;
    allRobots.insert(allRobots.end(), world.us.begin(), world.us.end());
    allRobots.insert(allRobots.end(), world.them.begin(), world.them.end());
    return allRobots;
}

} // ai
} // rtt