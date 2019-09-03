//
// Created by Lukas Bos on 30/08/2019.
//

#ifndef RTT_FIELDMESSAGE_H
#define RTT_FIELDMESSAGE_H

#include "roboteam_proto/FieldLineSegment.pb.h"
#include "roboteam_proto/FieldCircularArc.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include <roboteam_utils/Vector2.h>
namespace rtt {

struct FieldLineSegment {
  Vector2 begin;
  Vector2 end;
  std::string name;
  float thickness;
};

struct FieldArc {
  Vector2 center;
  float radius;
  float a1;
  float a2;
  std::string name;
  float thickness;
};


class FieldMessage {
 private:
  double width;
  double length;
  double goalWidth;
  double goalDepth;
  double boundaryWidth;

  // The field lines and arcs. For easy addressing.
  FieldLineSegment top_line;
  FieldLineSegment bottom_line;
  FieldLineSegment left_line;
  FieldLineSegment right_line;
  FieldLineSegment half_line;
  FieldLineSegment center_line;
  FieldLineSegment left_penalty_line;
  FieldLineSegment right_penalty_line;
  FieldArc top_left_penalty_arc;
  FieldArc bottom_left_penalty_arc;
  FieldArc top_right_penalty_arc;
  FieldArc bottom_right_penalty_arc;
  FieldLineSegment top_left_penalty_stretch;
  FieldLineSegment bottom_left_penalty_stretch;
  FieldLineSegment top_right_penalty_stretch;
  FieldLineSegment bottom_right_penalty_stretch;
  FieldArc center_circle;

  // All the field lines and arcs again. For easy iterating.
  std::vector<FieldLineSegment> field_lines;
  std::vector<FieldArc> field_arcs;

 public:
  FieldMessage() = default;
  FieldMessage(roboteam_proto::SSL_GeometryFieldSize sslFieldSize);
  void invert();
  float mm_to_m(float scalar);
  Vector2 mm_to_m(Vector2 vector);

  double field_width();
  double field_length();
  double goal_width();
  double goal_depth();
  double boundary_width();
  FieldLineSegment getTop_line();
  FieldLineSegment getBottom_line();
  FieldLineSegment getLeft_line();
  FieldLineSegment getRight_line();
  FieldLineSegment getHalf_line();
  FieldLineSegment getCenter_line();
  FieldLineSegment getLeft_penalty_line();
  FieldLineSegment getRight_penalty_line();
  FieldArc getTop_left_penalty_arc();
  FieldArc getBottom_left_penalty_arc();
  FieldArc getTop_right_penalty_arc();
  FieldArc getBottom_right_penalty_arc();
  FieldLineSegment getTop_left_penalty_stretch();
  FieldLineSegment getBottom_left_penalty_stretch();
  FieldLineSegment getTop_right_penalty_stretch();
  FieldLineSegment getBottom_right_penalty_stretch();
  FieldArc getCenter_circle();
  std::vector<FieldLineSegment> getField_lines();
  std::vector<FieldArc> getField_arcs();

  void addHelperLine(FieldLineSegment newLine);

 private:
  std::map<std::string, std::string> name_map = {
      std::make_pair("TopTouchLine", "top_line"),
      std::make_pair("BottomTouchLine", "bottom_line"),
      std::make_pair("LeftGoalLine", "left_line"),
      std::make_pair("RightGoalLine", "right_line"),
      std::make_pair("HalfwayLine", "half_line"),
      std::make_pair("CenterLine", "center_line"),
      std::make_pair("LeftPenaltyStretch", "left_penalty_line"),
      std::make_pair("RightPenaltyStretch", "right_penalty_line"),

      std::make_pair("LeftFieldLeftPenaltyArc", "top_left_penalty_arc"),
      std::make_pair("LeftFieldRightPenaltyArc", "bottom_left_penalty_arc"),
      std::make_pair("RightFieldLeftPenaltyArc", "top_right_penalty_arc"),
      std::make_pair("RightFieldRightPenaltyArc", "bottom_right_penalty_arc"),

      std::make_pair("LeftFieldLeftPenaltyStretch", "top_left_penalty_stretch"),
      std::make_pair("LeftFieldRightPenaltyStretch", "bottom_left_penalty_stretch"),
      std::make_pair("RightFieldLeftPenaltyStretch", "bottom_right_penalty_stretch"),
      std::make_pair("RightFieldRightPenaltyStretch", "top_right_penalty_stretch"),

      std::make_pair("CenterCircle", "center_circle"),
  };
  void addHelperArc(FieldArc newArc);

    void invertArc(FieldArc &arc) const;

    void invertFieldLine(FieldLineSegment &line) const;
};

}
#endif //RTT_FIELDMESSAGE_H
