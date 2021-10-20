//
// Created by robzelluf on 4/16/19.
//

#include "FieldHelper.h"

namespace testhelpers {

// generateField defaults to a A division field
proto::SSL_GeometryFieldSize FieldHelper::generateField(double field_length,
                                                        double field_width,
                                                        double goal_width,
                                                        double defense_area_width,
                                                        double defense_area_length,
                                                        double center_circle_radius) {
  proto::SSL_GeometryFieldSize field;

  field.set_field_length(field_length);
  field.set_field_width(field_width);
  field.set_goal_width(goal_width);

  addDefenseAreas(field, defense_area_width, defense_area_length);
  addCenterArc(field, center_circle_radius);
  return field;
}

void FieldHelper::addDefenseAreas(proto::SSL_GeometryFieldSize &field,
                                  double defenseAreaWidth,
                                  double defenseAreaDepth) {
  auto top_right_penalty_stretch_begin =
      rtt::Vector2{field.field_length() / 2.0 - defenseAreaDepth, defenseAreaWidth / 2};
  auto top_right_penalty_stretch_end = rtt::Vector2(field.field_length() / 2.0, defenseAreaWidth / 2);
  auto bottom_right_penalty_stretch_begin =
      rtt::Vector2(field.field_length() / 2.0 - defenseAreaDepth, -defenseAreaWidth / 2);
  auto bottom_right_penalty_stretch_end = rtt::Vector2(field.field_length() / 2.0, -defenseAreaWidth / 2);
  auto right_penalty_line_begin = rtt::Vector2(field.field_length() / 2.0 - defenseAreaDepth, -defenseAreaWidth / 2);
  auto right_penalty_line_end = rtt::Vector2(field.field_length() / 2.0 - defenseAreaDepth, defenseAreaWidth / 2);
  auto top_left_penalty_stretch_begin =
      rtt::Vector2(-field.field_length() / 2.0 + defenseAreaDepth, defenseAreaWidth / 2);
  auto top_left_penalty_stretch_end = rtt::Vector2(-field.field_length() / 2.0, defenseAreaWidth / 2);
  auto bottom_left_penalty_stretch_begin =
      rtt::Vector2(-field.field_length() / 2.0 + defenseAreaDepth, -defenseAreaWidth / 2);
  auto bottom_left_penalty_stretch_end = rtt::Vector2(-field.field_length() / 2.0, -defenseAreaWidth / 2);
  auto left_penalty_line_begin = rtt::Vector2(-field.field_length() / 2.0 + defenseAreaDepth, -defenseAreaWidth / 2);
  auto left_penalty_line_end = rtt::Vector2(-field.field_length() / 2.0 + defenseAreaDepth, defenseAreaWidth / 2);

  addLine(field, top_right_penalty_stretch_begin, top_right_penalty_stretch_end, proto::RightFieldRightPenaltyStretch);
  addLine(field,
          bottom_right_penalty_stretch_begin,
          bottom_right_penalty_stretch_end,
          proto::RightFieldLeftPenaltyStretch);
  addLine(field, top_left_penalty_stretch_begin, top_left_penalty_stretch_end, proto::LeftFieldLeftPenaltyStretch);
  addLine(field,
          bottom_left_penalty_stretch_begin,
          bottom_left_penalty_stretch_end,
          proto::LeftFieldRightPenaltyStretch);
  addLine(field, left_penalty_line_begin, left_penalty_line_end, proto::LeftPenaltyStretch);
  addLine(field, right_penalty_line_begin, right_penalty_line_end, proto::RightPenaltyStretch);
}

void FieldHelper::addCenterArc(proto::SSL_GeometryFieldSize &field, double radius) {
  proto::SSL_FieldCircularArc center_circle;
  center_circle.mutable_center()->set_x(0);
  center_circle.mutable_center()->set_y(0);
  center_circle.set_radius(radius);
  center_circle.set_a1(0.0);
  center_circle.set_a2(2 * M_PI);
  center_circle.set_type(proto::CenterCircle);
  field.add_field_arcs()->CopyFrom(center_circle);
}
void FieldHelper::addLine(proto::SSL_GeometryFieldSize &field, Vector2 begin,
                          Vector2 end, proto::SSL_FieldShapeType type) {
  proto::SSL_FieldLineSegment line;

  line.mutable_p1()->set_x(begin.x);
  line.mutable_p1()->set_y(begin.y);
  line.mutable_p2()->set_x(end.x);
  line.mutable_p2()->set_y(end.y);
  line.set_type(type);
  line.set_thickness(0.01);//Default thickness
  field.add_field_lines()->CopyFrom(line);
}

}  // namespace testhelpers
