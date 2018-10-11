#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
roboteam_msgs::GeometryFieldSize World::field;

const roboteam_msgs::World &World::get_world() {
  return World::world;
}

void World::set_world(roboteam_msgs::World world) {
  World::world = world;
}

const roboteam_msgs::GeometryFieldSize World::get_field() {
  return World::field;
}

void World::set_field(roboteam_msgs::GeometryFieldSize field) {
  World::field = field;
}

} // ai
} // rtt