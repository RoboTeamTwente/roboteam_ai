//
// Created by robzelluf on 4/16/19.
//

#include "FieldHelper.h"

namespace testhelpers {

// generateField defaults to a A division field
    roboteam_msgs::GeometryFieldSize FieldHelper::generateField(double field_length, double field_width, double goal_width, double defense_area_width, double defense_area_length, double center_circle_radius) {
    roboteam_msgs::GeometryFieldSize field;

    field.field_length = field_length;
    field.field_width = field_width;
    field.goal_width = goal_width;

    addDefenseAreas(field, defense_area_width, defense_area_length);
    addCenterArc(field, center_circle_radius);
    return field;
}

void FieldHelper::addDefenseAreas(roboteam_msgs::GeometryFieldSize &field, double defenseAreaWidth, double defenseAreaDepth) {
    field.top_right_penalty_stretch.begin = Vector2{field.field_length / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    field.top_right_penalty_stretch.end = Vector2{field.field_length / 2, defenseAreaWidth / 2};
    field.bottom_right_penalty_stretch.begin = field.top_right_penalty_stretch.begin;
    field.bottom_right_penalty_stretch.end = Vector2{field.field_length / 2, -defenseAreaWidth / 2};

    field.top_left_penalty_stretch.begin = Vector2{-field.field_length / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    field.top_left_penalty_stretch.end = Vector2{-field.field_length / 2, defenseAreaWidth / 2};
    field.bottom_left_penalty_stretch.begin = field.top_left_penalty_stretch.begin;
    field.bottom_left_penalty_stretch.end = Vector2{-field.field_length / 2, -defenseAreaWidth / 2};

    field.left_penalty_line.begin = Vector2{(-field.field_length / 2) + defenseAreaDepth, defenseAreaWidth};
    field.left_penalty_line.end = Vector2{(-field.field_length / 2) + defenseAreaDepth, -defenseAreaWidth};
    field.right_penalty_line.begin = Vector2{(field.field_length / 2) - defenseAreaDepth, defenseAreaWidth};
    field.right_penalty_line.end = Vector2{(field.field_length / 2) - defenseAreaDepth, -defenseAreaWidth};
}

void FieldHelper::addCenterArc(roboteam_msgs::GeometryFieldSize &field, double radius) {
    field.center_circle.center = Vector2{0, 0};
    field.center_circle.radius = radius;
}

}
