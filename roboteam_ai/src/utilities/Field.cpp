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

} // ai
} // rtt