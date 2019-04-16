//
// Created by robzelluf on 4/16/19.
//

#include "FieldHelper.h"

namespace testhelpers {

roboteam_msgs::GeometryFieldSize FieldHelper::getDivisionAField() {
    roboteam_msgs::GeometryFieldSize field;

    field.field_length = 12;
    field.field_width = 9;
    field.goal_width = 1.2;

    double defenseAreaWidth = 2.4;
    double defenseAreaDepth = 1.2;

    field.top_right_penalty_stretch.begin = Vector2{field.field_length / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    field.top_right_penalty_stretch.end = Vector2{field.field_length / 2, defenseAreaWidth / 2};
    field.bottom_right_penalty_stretch.begin = field.top_right_penalty_stretch.begin;
    field.bottom_right_penalty_stretch.end = Vector2{field.field_length / 2, -defenseAreaWidth / 2};

    field.top_left_penalty_stretch.begin = Vector2{-field.field_length / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    field.top_left_penalty_stretch.end = Vector2{-field.field_length / 2, defenseAreaWidth / 2};
    field.bottom_left_penalty_stretch.begin = field.top_left_penalty_stretch.begin;
    field.bottom_left_penalty_stretch.end = Vector2{-field.field_length / 2, -defenseAreaWidth / 2};

    return field;
}

}
