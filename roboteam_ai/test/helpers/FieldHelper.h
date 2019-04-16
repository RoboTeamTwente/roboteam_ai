//
// Created by robzelluf on 4/16/19.
//

#ifndef ROBOTEAM_AI_FIELDHELPER_H
#define ROBOTEAM_AI_FIELDHELPER_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/GeometryFieldSize.h>
#include <roboteam_ai/src/world/Field.h>

namespace testhelpers {

class FieldHelper {
public:
    static roboteam_msgs::GeometryFieldSize getDivisionAField();
};

}


#endif //ROBOTEAM_AI_FIELDHELPER_H
