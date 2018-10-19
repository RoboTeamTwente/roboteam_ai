//
// Created by mrlukasbos on 19-10-18.
//

#include "Field.h"

namespace rtt {
namespace ai {

roboteam_msgs::GeometryFieldSize Field::field;

const roboteam_msgs::GeometryFieldSize Field::get_field() {
  return Field::field;
}

void Field::set_field(roboteam_msgs::GeometryFieldSize field) {
  Field::field = field;
}

Vector2 Field::get_our_goal_center() {
  return Vector2(field.field_length / -2, 0);
}

Vector2 Field::get_their_goal_center() {
  return Vector2(field.field_length / 2, 0);
}

bool Field::pointIsInDefenceArea(Vector2 point, bool isOurDefenceArea, float margin) {
  double xBound;
  double yTopBound;
  double yBottomBound;
  if (isOurDefenceArea) {
    xBound = field.left_penalty_line.begin.x;
    if (field.left_penalty_line.begin.y < field.left_penalty_line.end.y) {
      yBottomBound = field.left_penalty_line.begin.y;
      yTopBound = field.left_penalty_line.end.y;
    } else {
      yBottomBound = field.left_penalty_line.end.y;
      yTopBound = field.left_penalty_line.begin.y;
    }
    if (point.x < (xBound + margin) && point.y<(yTopBound + margin) && point.y>(yBottomBound - margin)) {
      return true;
    } else return false;
  } else { // their defense area
    xBound = field.right_penalty_line.begin.x;
    if (field.right_penalty_line.begin.y < field.right_penalty_line.end.y) {
      yBottomBound = field.right_penalty_line.begin.y;
      yTopBound = field.right_penalty_line.end.y;
    } else {
      yBottomBound = field.right_penalty_line.end.y;
      yTopBound = field.right_penalty_line.begin.y;
    }
    if (point.x > (xBound - margin) && point.y<(yTopBound + margin) && point.y>(yBottomBound - margin)) {
      return true;
    } else return false;
  }
}

} // ai
} // rtt