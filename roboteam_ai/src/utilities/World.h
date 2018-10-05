//
// Created by mrlukasbos on 5-10-18.
//

#ifndef ROBOTEAM_AI_WORLD_H
#define ROBOTEAM_AI_WORLD_H

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryData.h"

namespace rtt {
namespace ai {

class World {
 private:
  static roboteam_msgs::World world;
  static roboteam_msgs::GeometryFieldSize field;

 public:
  // getters and setters for the world
  static const roboteam_msgs::World& get_world();
  static void set_world(roboteam_msgs::World world);
  static const roboteam_msgs::GeometryFieldSize get_field();
  static void set_field(roboteam_msgs::GeometryFieldSize field);
};


} // ai
} // rtt

#endif //ROBOTEAM_AI_WORLD_H
