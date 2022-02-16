//
// Created by robzelluf on 4/16/19.
//

#ifndef ROBOTEAM_AI_FIELDHELPER_H
#define ROBOTEAM_AI_FIELDHELPER_H

#include <proto/messages_robocup_ssl_geometry.pb.h>
#include <roboteam_utils/Vector2.h>

#include "world/FieldComputations.h"

namespace testhelpers {

class FieldHelper {
   public:
    static proto::SSL_GeometryFieldSize generateField(double field_length = 12000, double field_width = 9000, double goal_width = 1200, double defense_area_width = 2400,
                                                      double defense_area_depth = 1200, double center_circle_radius = 500);
    static void addDefenseAreas(proto::SSL_GeometryFieldSize &field, double defenseAreaWidth, double defenseAreaDepth);
    static void addLine(proto::SSL_GeometryFieldSize &field, Vector2 begin, Vector2 end, proto::SSL_FieldShapeType type);
    static void addCenterArc(proto::SSL_GeometryFieldSize &field, double radius = 0.05);
};

}  // namespace testhelpers

#endif  // ROBOTEAM_AI_FIELDHELPER_H
