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
    static roboteam_msgs::GeometryFieldSize generateField(double field_length = 12.0, double field_width = 9.0, double goal_width = 1.2, double defense_area_width = 2.4, double defense_area_depth = 1.2, double center_circle_radius = 0.05);
    static void addDefenseAreas(roboteam_msgs::GeometryFieldSize &field, double defenseAreaWidth,
                                double defenseAreaDepth);
    static void addCenterArc(roboteam_msgs::GeometryFieldSize &field, double radius = 0.05);
};

}


#endif //ROBOTEAM_AI_FIELDHELPER_H
