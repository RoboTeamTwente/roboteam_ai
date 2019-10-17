//
// Created by Lukas Bos on 30/08/2019.
//

#include <include/roboteam_ai/world/FieldMessage.h>
namespace rtt {

FieldMessage::FieldMessage(roboteam_proto::SSL_GeometryFieldSize sslFieldSize) {
  length = mm_to_m(sslFieldSize.field_length());
  width = mm_to_m(sslFieldSize.field_width());
  goalWidth = mm_to_m(sslFieldSize.goal_width());
  goalDepth = mm_to_m(sslFieldSize.goal_depth());
  boundaryWidth = mm_to_m(sslFieldSize.boundary_width());

  for (roboteam_proto::SSL_FieldLineSegment line : sslFieldSize.field_lines()) {
    FieldLineSegment newLine;
    newLine.name = std::string(name_map[line.name()]);
    newLine.begin = mm_to_m(line.p1());
    newLine.end = mm_to_m(line.p2());
    newLine.thickness = mm_to_m(line.thickness());

    addHelperLine(newLine);
    field_lines.push_back(newLine);
  }
  
  for (roboteam_proto::SSL_FieldCicularArc arc : sslFieldSize.field_arcs()) {
    FieldArc newArc;
    newArc.name = std::string(name_map[arc.name()]);
    newArc.center = mm_to_m(arc.center());
    newArc.a1 = mm_to_m(arc.a1());
    newArc.a2 = mm_to_m(arc.a2());
    newArc.radius = mm_to_m(arc.radius());
    newArc.thickness = mm_to_m(arc.thickness());

    addHelperArc(newArc);
    field_arcs.push_back(newArc);
  }

}

void FieldMessage::addHelperArc(FieldArc newArc) {
  if (newArc.name == "top_left_penalty_arc") {
      top_left_penalty_arc = newArc;
    } else if (newArc.name == "bottom_left_penalty_arc") {
      bottom_left_penalty_arc =newArc;
    } else if (newArc.name == "top_right_penalty_arc") {
      top_right_penalty_arc = newArc;
    } else if (newArc.name == "bottom_right_penalty_arc") {
      bottom_right_penalty_arc = newArc;
    } else if (newArc.name == "center_circle") {
      center_circle = newArc;
    }
}

void FieldMessage::addHelperLine(FieldLineSegment newLine) {
  if (newLine.name == "top_line") {
      top_line = newLine;
    } else if (newLine.name == "bottom_line") {
      bottom_line = newLine;
    } else if (newLine.name == "left_line") {
      left_line = newLine;
    } else if (newLine.name == "right_line") {
      right_line = newLine;
    } else if (newLine.name == "half_line") {
      half_line = newLine;
    } else if (newLine.name == "center_line") {
      center_line = newLine;
    } else if (newLine.name == "left_penalty_line") {
      left_penalty_line = newLine;
    } else if (newLine.name == "right_penalty_line") {
      right_penalty_line = newLine;
    }
      // adding rectangle box lines
    else if (newLine.name == "top_left_penalty_stretch") {
      top_left_penalty_stretch = newLine;
    } else if (newLine.name == "bottom_left_penalty_stretch") {
      bottom_left_penalty_stretch = newLine;
    } else if (newLine.name == "top_right_penalty_stretch") {
      top_right_penalty_stretch = newLine;
    } else if (newLine.name == "bottom_right_penalty_stretch") {
      bottom_right_penalty_stretch = newLine;
    }
}

float FieldMessage::mm_to_m(float scalar) {
  return scalar/1000;
}

Vector2 FieldMessage::mm_to_m(Vector2 vector) {
  return {vector.x/1000, vector.y/1000};
}

// getters And setters
double FieldMessage::field_width(){
  return width;
}
double FieldMessage::field_length(){
  return length;
}
double FieldMessage::goal_width(){
  return goalWidth;
}
double FieldMessage::goal_depth(){
  return goalDepth;
}
double FieldMessage::boundary_width(){
  return boundaryWidth;
}
double FieldMessage::getLeftLineX() {
    FieldLineSegment left_line = FieldMessage::getLeft_line();
    return left_line.begin.x;
}

FieldLineSegment FieldMessage::getTop_line(){
  return top_line;
}
FieldLineSegment FieldMessage::getBottom_line(){
  return bottom_line;
}
FieldLineSegment FieldMessage::getLeft_line(){
  return left_line;
}
double FieldMessage::getLeftLineX() {
    FieldLineSegment left_line = FieldMessage::getLeft_line();
    return left_line.begin.x;
}
FieldLineSegment FieldMessage::getRight_line(){
  return right_line;
}
FieldLineSegment FieldMessage::getHalf_line(){
  return half_line;
}
FieldLineSegment FieldMessage::getCenter_line(){
  return center_line;
}
FieldLineSegment FieldMessage::getLeft_penalty_line(){
  return left_penalty_line;
}
FieldLineSegment FieldMessage::getRight_penalty_line(){
  return right_penalty_line;
}
FieldArc  FieldMessage::getTop_left_penalty_arc(){
  return top_left_penalty_arc;
}
FieldArc  FieldMessage::getBottom_left_penalty_arc(){
  return bottom_left_penalty_arc;
}
FieldArc  FieldMessage::getTop_right_penalty_arc(){
  return top_right_penalty_arc;
}
FieldArc  FieldMessage::getBottom_right_penalty_arc(){
  return bottom_right_penalty_arc;
}
FieldLineSegment FieldMessage::getTop_left_penalty_stretch(){
  return top_left_penalty_stretch;
}
FieldLineSegment FieldMessage::getBottom_left_penalty_stretch(){
  return bottom_left_penalty_stretch;
}
FieldLineSegment FieldMessage::getTop_right_penalty_stretch(){
  return top_right_penalty_stretch;
}
FieldLineSegment FieldMessage::getBottom_right_penalty_stretch(){
  return bottom_right_penalty_stretch;
}
FieldArc FieldMessage::getCenter_circle(){
  return center_circle;
}

std::vector<FieldLineSegment> FieldMessage::getField_lines(){
  return field_lines;
}
std::vector<FieldArc> FieldMessage::getField_arcs(){
  return field_arcs;
}

void FieldMessage::invert() {

    for (auto line : field_lines) {
        invertFieldLine(line);
    }

//    invertFieldLine(top_line);
//    invertFieldLine(bottom_line);
//    invertFieldLine(left_line);
//    invertFieldLine(right_line);
//    invertFieldLine(half_line);
//    invertFieldLine(center_line);
//    invertFieldLine(left_penalty_line);
//    invertFieldLine(right_penalty_line);
//
//    invertFieldLine(top_left_penalty_stretch);
//    invertFieldLine(bottom_left_penalty_stretch);
//    invertFieldLine(top_right_penalty_stretch);
//    invertFieldLine(bottom_right_penalty_stretch);
//
//
//    invertArc(top_left_penalty_arc);
//    invertArc(bottom_left_penalty_arc);
//    invertArc(top_right_penalty_arc);
//    invertArc(bottom_right_penalty_arc);
//    invertArc(center_circle);

    for (auto arc : field_arcs) {
        invertArc(arc);
    }
}

    void FieldMessage::invertFieldLine(FieldLineSegment &line) const {
        line.begin.x = -line.begin.x;
        line.begin.y = -line.begin.y;
        line.end.x = -line.end.x;
        line.end.y = -line.end.y;
    }

    void FieldMessage::invertArc(FieldArc &arc) const {
        arc.center.x = -arc.center.x;
        arc.center.y = -arc.center.y;
        arc.a1 = -arc.a1;
        arc.a2 = -arc.a2;
    }

}