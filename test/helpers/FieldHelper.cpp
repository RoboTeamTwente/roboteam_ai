//
// Created by robzelluf on 4/16/19.
//

#include "FieldHelper.h"

namespace testhelpers {

// generateField defaults to a A division field
proto::GeometryFieldSize FieldHelper::generateField(double field_length, double field_width, double goal_width, double defense_area_width, double defense_area_length,
                                                    double center_circle_radius) {
    proto::GeometryFieldSize field;

    field.set_field_length(field_length);
    field.set_field_width(field_width);
    field.set_goal_width(goal_width);

    addDefenseAreas(field, defense_area_width, defense_area_length);
    addCenterArc(field, center_circle_radius);
    return field;
}

void FieldHelper::addDefenseAreas(proto::GeometryFieldSize &field, double defenseAreaWidth, double defenseAreaDepth) {
    auto top_right_penalty_stretch_begin = rtt::Vector2{field.field_length() / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    auto top_right_penalty_stretch_end = rtt::Vector2{field.field_length() / 2, defenseAreaWidth / 2};
    auto bottom_right_penalty_stretch_begin = rtt::Vector2{field.field_length() / 2 - defenseAreaDepth, -defenseAreaWidth / 2};
    auto bottom_right_penalty_stretch_end = rtt::Vector2{field.field_length() / 2, -defenseAreaWidth / 2};
    auto right_penalty_line_begin = rtt::Vector2{field.field_length() / 2 - defenseAreaDepth, -defenseAreaWidth / 2};
    auto right_penalty_line_end = rtt::Vector2{field.field_length() / 2 - defenseAreaDepth, defenseAreaWidth / 2};
    auto top_left_penalty_stretch_begin = rtt::Vector2{-field.field_length() / 2 + defenseAreaDepth, defenseAreaWidth / 2};
    auto top_left_penalty_stretch_end = rtt::Vector2{-field.field_length() / 2, defenseAreaWidth / 2};
    auto bottom_left_penalty_stretch_begin = rtt::Vector2{-field.field_length() / 2 + defenseAreaDepth, -defenseAreaWidth / 2};
    auto bottom_left_penalty_stretch_end = rtt::Vector2{-field.field_length() / 2, -defenseAreaWidth / 2};
    auto left_penalty_line_begin = rtt::Vector2{-field.field_length() / 2 + defenseAreaDepth, -defenseAreaWidth / 2};
    auto left_penalty_line_end = rtt::Vector2{-field.field_length() / 2 + defenseAreaDepth, defenseAreaWidth / 2};

    field.mutable_top_right_penalty_stretch()->mutable_begin()->set_x(top_right_penalty_stretch_begin.x);
    field.mutable_top_right_penalty_stretch()->mutable_begin()->set_y(top_right_penalty_stretch_begin.y);
    field.mutable_top_right_penalty_stretch()->mutable_end()->set_x(top_right_penalty_stretch_end.x);
    field.mutable_top_right_penalty_stretch()->mutable_end()->set_y(top_right_penalty_stretch_end.y);

    field.mutable_top_left_penalty_stretch()->mutable_begin()->set_x(top_left_penalty_stretch_begin.x);
    field.mutable_top_left_penalty_stretch()->mutable_begin()->set_y(top_left_penalty_stretch_begin.y);
    field.mutable_top_left_penalty_stretch()->mutable_end()->set_x(top_left_penalty_stretch_end.x);
    field.mutable_top_left_penalty_stretch()->mutable_end()->set_y(top_left_penalty_stretch_end.y);

    field.mutable_bottom_right_penalty_stretch()->mutable_begin()->set_x(bottom_right_penalty_stretch_begin.x);
    field.mutable_bottom_right_penalty_stretch()->mutable_begin()->set_y(bottom_right_penalty_stretch_begin.y);
    field.mutable_bottom_right_penalty_stretch()->mutable_end()->set_x(bottom_right_penalty_stretch_end.x);
    field.mutable_bottom_right_penalty_stretch()->mutable_end()->set_y(bottom_right_penalty_stretch_end.y);

    field.mutable_bottom_left_penalty_stretch()->mutable_begin()->set_x(bottom_left_penalty_stretch_begin.x);
    field.mutable_bottom_left_penalty_stretch()->mutable_begin()->set_y(bottom_left_penalty_stretch_begin.y);
    field.mutable_bottom_left_penalty_stretch()->mutable_end()->set_x(bottom_left_penalty_stretch_end.x);
    field.mutable_bottom_left_penalty_stretch()->mutable_end()->set_y(bottom_left_penalty_stretch_end.y);

    field.mutable_left_penalty_line()->mutable_begin()->set_x(left_penalty_line_begin.x);
    field.mutable_left_penalty_line()->mutable_begin()->set_y(left_penalty_line_begin.y);
    field.mutable_left_penalty_line()->mutable_end()->set_x(left_penalty_line_end.x);
    field.mutable_left_penalty_line()->mutable_end()->set_y(left_penalty_line_end.y);

    field.mutable_right_penalty_line()->mutable_begin()->set_x(right_penalty_line_begin.x);
    field.mutable_right_penalty_line()->mutable_begin()->set_y(right_penalty_line_begin.y);
    field.mutable_right_penalty_line()->mutable_end()->set_x(right_penalty_line_end.x);
    field.mutable_right_penalty_line()->mutable_end()->set_y(right_penalty_line_end.y);
}

void FieldHelper::addCenterArc(proto::GeometryFieldSize &field, double radius) {
    field.mutable_center_circle()->mutable_center()->set_x(0);
    field.mutable_center_circle()->mutable_center()->set_y(0);
    field.mutable_center_circle()->set_radius(radius);
}

}  // namespace testhelpers
