#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
bool World::didReceiveFirstWorld = false;

const roboteam_msgs::World &World::get_world() {
    return World::world;
}

void World::set_world(roboteam_msgs::World world) {
    if (! world.us.empty()) {
        didReceiveFirstWorld = true;
    }
    World::world = world;
}

std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotForId(unsigned int id, bool robotIsOurTeam) {
    const std::vector<roboteam_msgs::WorldRobot> &robots = robotIsOurTeam ? world.us : world.them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return std::make_shared<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return nullptr;
}

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

roboteam_msgs::WorldBall World::getBall() {
    return world.ball;
}

bool World::bot_has_ball(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball) {
    Vector2 ball_vec(ball.pos), bot_vec(bot.pos);
    Vector2 ball_norm = (ball_vec - bot_vec);

    double dist = ball_norm.length();
    double angle = ball_norm.angle();

    //TODO: Check if the angle taken this way does not fail because of angle jump at pi or 2 pi (it should)
    //TODO: TEST if this is from centre of dribbler of robot in practice. What does
    // Within 15 cm and .4 radians (of center of dribbler)
    return dist <= .15 && fabs(angle - bot.angle) <= .4;
}

std::vector<roboteam_msgs::WorldRobot> World::getAllRobots() {
    std::vector<roboteam_msgs::WorldRobot> allRobots;
    allRobots.insert(allRobots.end(), world.us.begin(), world.us.end());
    allRobots.insert(allRobots.end(), world.them.begin(), world.them.end());
    return allRobots;
}

} // ai
} // rtt