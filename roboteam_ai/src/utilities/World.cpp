#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
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

    if (! _world.us.empty()) {
        didReceiveFirstWorld = true;
    }
    if(! _world.ball.visible){
        _world.ball=world.ball;
        _world.ball.visible=false;
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
            robots.push_back(* robot);
        }
    }
    return robots;
}

/// returns the robot from a given vector closest to a given point
std::shared_ptr<int> World::get_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point) {

    int closest_robot = - 1;
    double closest_robot_ds = std::numeric_limits<double>::max();
    for (roboteam_msgs::WorldRobot worldRobot : robots) {
        Vector2 pos(worldRobot.pos);
        if ((pos - point).length() < closest_robot_ds) {
            closest_robot = worldRobot.id;
            closest_robot_ds = (pos - point).length();
        }
    }
    return closest_robot == - 1 ? nullptr : std::make_shared<int>(closest_robot);
}

/// returns the ball msg
std::shared_ptr<roboteam_msgs::WorldBall> World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return std::make_shared<roboteam_msgs::WorldBall>(world.ball);
}

/// returns boolean if a robot for a given ID has the ball
bool World::bot_has_ball(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball) {
    std::lock_guard<std::mutex> lock(worldMutex);
    Vector2 ball_vec(ball.pos), bot_vec(bot.pos);
    Vector2 ball_norm = (ball_vec - bot_vec);
    double dist = ball_norm.length();
    double angle = ball_norm.angle();

    //TODO: Check if the angle taken this way does not fail because of angle jump at pi or 2 pi (it should)
    //TODO: TEST if this is from centre of dribbler of robot in practice. What does
    // Within 15 cm and .4 radians (of center of dribbler)
    return dist <= .15 && fabs(angle - bot.angle) <= .4;
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