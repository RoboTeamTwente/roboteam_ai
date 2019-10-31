//
// Created by Lukas Bos on 30/08/2019.
//

#include <include/roboteam_ai/world/FieldMessage.h>
namespace rtt {

FieldMessage::FieldMessage(roboteam_proto::SSL_GeometryFieldSize sslFieldSize) {
    fieldValues[FIELD_LENGTH] = mm_to_m(sslFieldSize.field_length());
    fieldValues[FIELD_WIDTH] = mm_to_m(sslFieldSize.field_width());
    fieldValues[GOAL_WIDTH] = mm_to_m(sslFieldSize.goal_width());
    fieldValues[GOAL_DEPTH] = mm_to_m(sslFieldSize.goal_depth());
    fieldValues[BOUNDARY_WIDTH] = mm_to_m(sslFieldSize.boundary_width());

    for (roboteam_proto::SSL_FieldLineSegment line : sslFieldSize.field_lines()) {
        FieldLineSegment newLine;
        if (NAME_MAP.count(line.name()) > 0) {
            newLine.name = std::string(NAME_MAP[line.name()]);
            newLine.begin = mm_to_m(line.p1());
            newLine.end = mm_to_m(line.p2());
            newLine.thickness = mm_to_m(line.thickness());
            FieldLineName fieldLineName = CONVERT_TO_FIELD_LINE_NAME.at(newLine.name);
            fieldLines[fieldLineName] = newLine;
        }
    }

    for (roboteam_proto::SSL_FieldCicularArc arc : sslFieldSize.field_arcs()) {
        FieldArc newArc;
        if (NAME_MAP.count(arc.name()) > 0) {
            newArc.name = std::string(NAME_MAP[arc.name()]);
            newArc.center = mm_to_m(arc.center());
            newArc.a1 = mm_to_m(arc.a1());
            newArc.a2 = mm_to_m(arc.a2());
            newArc.radius = mm_to_m(arc.radius());
            newArc.thickness = mm_to_m(arc.thickness());
            FieldArcName fieldArcName = CONVERT_TO_FIELD_ARC_NAME.at(newArc.name);
            fieldArcs[fieldArcName] = newArc;
            field_arcs.push_back(newArc);
        }
    }
    fieldValues[LEFTMOST_X] = fieldLines[LEFT_LINE].begin.x;
}

float FieldMessage::mm_to_m(float scalar) {
  return scalar/1000;
}

Vector2 FieldMessage::mm_to_m(Vector2 vector) {
  return {vector.x / 1000, vector.y / 1000};
}

double FieldMessage::get(FieldValueName valueName) {
    if (fieldValues.count(valueName) > 0) {
        return fieldValues.at(valueName);
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. So the values are equal to 0.0 */
        return 0.0;
    }
}

FieldLineSegment FieldMessage::get(FieldLineName lineName) {
    return fieldLines.at(lineName);
}

FieldArc FieldMessage::get(FieldArcName arcName) {
    return fieldArcs.at(arcName);
}

FieldLineSegment FieldMessage::getLeft_line(){
    return left_line;
}
FieldLineSegment FieldMessage::getRight_line(){
    return right_line;
}
FieldLineSegment FieldMessage::getLeft_penalty_line(){
    return fieldLines[LEFT_PENALTY_LINE];
}
FieldLineSegment FieldMessage::getRight_penalty_line(){
    return fieldLines[RIGHT_PENALTY_LINE];
}
FieldLineSegment FieldMessage::getTop_right_penalty_stretch(){
    return fieldLines[TOP_RIGHT_PENALTY_STRETCH];
}
FieldArc FieldMessage::getCenter_circle(){
    return fieldArcs[CENTER_CIRCLE];
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