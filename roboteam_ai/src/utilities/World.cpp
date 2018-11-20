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
    if (!world.us.empty()) {
        didReceiveFirstWorld = true;
    }
    World::world = world;
}

boost::optional<roboteam_msgs::WorldRobot> World::getRobotForId(unsigned int id, bool robotIsOurTeam) {
    const std::vector<roboteam_msgs::WorldRobot> &robots = robotIsOurTeam ? world.us : world.them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return boost::optional<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return boost::none;
}

roboteam_msgs::WorldBall World::getBall() {
    return world.ball;
}

} // ai
} // rtt