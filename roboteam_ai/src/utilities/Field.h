/*
 * Field.h
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */


#ifndef ROBOTEAM_AI_FIELD_H
#define ROBOTEAM_AI_FIELD_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {
namespace ai {

class Field {
 private:
  static roboteam_msgs::GeometryFieldSize field;

 public:
  static const roboteam_msgs::GeometryFieldSize get_field();
  static void set_field(roboteam_msgs::GeometryFieldSize field);
  static Vector2 get_our_goal_center();
  static Vector2 get_their_goal_center();
  bool pointIsInDefenceArea(Vector2 point);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
