#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;

const roboteam_msgs::World &World::get_world() {
  return World::world;
}

void World::set_world(roboteam_msgs::World world) {
  World::world = world;
}

boost::optional<roboteam_msgs::WorldRobot> World::getRobotForId(int id, bool robotIsOurTeam) {
  const std::vector<roboteam_msgs::WorldRobot>& robots = robotIsOurTeam ? world.us : world.them;
  for (const auto& bot : robots) {
    if (bot.id == id) {
      return boost::optional<roboteam_msgs::WorldRobot>(bot);
    }
  }
  return boost::none;
}

boost::optional<int> World::get_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point) {
    int closest_robot = -1;
    double closest_robot_ds = std::numeric_limits<double>::max();

    for (roboteam_msgs::WorldRobot worldRobot : robots) {
        Vector2 pos(worldRobot.pos);

        if ((pos - point).length() < closest_robot_ds) {
            closest_robot = worldRobot.id;
            closest_robot_ds = (pos - point).length();
        }
    }

    return closest_robot == -1 ? boost::none : boost::optional<int>(closest_robot);
}

roboteam_msgs::WorldBall World::getBall() {
  return world.ball;
}
} // ai
} // rtt