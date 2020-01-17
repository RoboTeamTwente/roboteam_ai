//
// Created by Lukas Bos on 30/08/2019.
//

#include <include/roboteam_ai/world/FieldMessage.h>
namespace rtt {

FieldMessage::FieldMessage(proto::SSL_GeometryFieldSize sslFieldSize) {
    initFieldLines(sslFieldSize);
    initFieldArcs(sslFieldSize);
    initFieldValues(sslFieldSize);
    initFieldVectors();
}

void FieldMessage::initFieldValues(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    fieldValues[FIELD_LENGTH] = mm_to_m(sslFieldSize.field_length());
    fieldValues[FIELD_WIDTH] = mm_to_m(sslFieldSize.field_width());
    fieldValues[GOAL_WIDTH] = mm_to_m(sslFieldSize.goal_width());
    fieldValues[GOAL_DEPTH] = mm_to_m(sslFieldSize.goal_depth());
    fieldValues[BOUNDARY_WIDTH] = mm_to_m(sslFieldSize.boundary_width());
    fieldValues[LEFTMOST_X] = -0.5 * fieldValues[FIELD_LENGTH];
    fieldValues[RIGHTMOST_X] = 0.5 * fieldValues[FIELD_LENGTH];
    fieldValues[BOTTOMMOST_Y] = -0.5 * fieldValues[FIELD_WIDTH];
    fieldValues[TOPMOST_Y] = 0.5 * fieldValues[FIELD_WIDTH];
    fieldValues[CENTER_Y] = 0.0;
}

void FieldMessage::initFieldLines(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    for (proto::SSL_FieldLineSegment line : sslFieldSize.field_lines()) {
        FieldLineSegment newLine;
        if (NAME_MAP.find(line.name()) != NAME_MAP.end()) {
            newLine.name = NAME_MAP[line.name()];
            newLine.begin = mm_to_m(line.p1());
            newLine.end = mm_to_m(line.p2());
            newLine.thickness = mm_to_m(line.thickness());
            FieldLineName fieldLineName = CONVERT_TO_FIELD_LINE_NAME.at(newLine.name);
            fieldLines[fieldLineName] = newLine;
        }
    }
}

void FieldMessage::initFieldArcs(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    for (proto::SSL_FieldCicularArc arc : sslFieldSize.field_arcs()) {
        FieldArc newArc;
        if (NAME_MAP.find(arc.name()) != NAME_MAP.end()) {
            newArc.name = NAME_MAP[arc.name()];
            newArc.center = mm_to_m(arc.center());
            newArc.a1 = mm_to_m(arc.a1());
            newArc.a2 = mm_to_m(arc.a2());
            newArc.radius = mm_to_m(arc.radius());
            newArc.thickness = mm_to_m(arc.thickness());
            FieldArcName fieldArcName = CONVERT_TO_FIELD_ARC_NAME.at(newArc.name);
            fieldArcs[fieldArcName] = newArc;
        }
    }
}

void FieldMessage::initFieldVectors() {
    fieldVectors[OUR_GOAL_CENTER] = Vector2(fieldValues[LEFTMOST_X], fieldValues[CENTER_Y]);
    fieldVectors[THEIR_GOAL_CENTER] = Vector2(fieldValues[RIGHTMOST_X], fieldValues[CENTER_Y]);

    Vector2 goalWidthAdjust = Vector2(0, fieldValues[GOAL_WIDTH] / 2);
    fieldVectors[OUR_BOTTOM_GOAL_SIDE] = fieldVectors[OUR_GOAL_CENTER] - goalWidthAdjust;
    fieldVectors[OUR_TOP_GOAL_SIDE] = fieldVectors[OUR_GOAL_CENTER] + goalWidthAdjust;
    fieldVectors[THEIR_BOTTOM_GOAL_SIDE] = fieldVectors[THEIR_GOAL_CENTER] - goalWidthAdjust;
    fieldVectors[THEIR_TOP_GOAL_SIDE] = fieldVectors[THEIR_GOAL_CENTER] + goalWidthAdjust;

    Vector2 lpl_begin = fieldLines[LEFT_PENALTY_LINE].begin;
    Vector2 lpl_end = fieldLines[LEFT_PENALTY_LINE].end;
    fieldVectors[LEFT_PENALTY_POINT] = lpl_begin + ((lpl_end - lpl_begin) * 0.5);

    Vector2 rpl_begin = fieldLines[RIGHT_PENALTY_LINE].begin;
    Vector2 rpl_end = fieldLines[RIGHT_PENALTY_LINE].end;
    fieldVectors[RIGHT_PENALTY_POINT] = rpl_begin + ((rpl_end - rpl_begin) * 0.5);
}

float FieldMessage::mm_to_m(float scalar) { return scalar / 1000; }

Vector2 FieldMessage::mm_to_m(Vector2 vector) { return {vector.x / 1000, vector.y / 1000}; }

double FieldMessage::get(FieldValueName valueName) const {
    if (fieldValues.find(valueName) != fieldValues.end()) {
        return fieldValues.at(valueName);
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. So the values are equal to 0.0 */
//        std::cout << "Access undefined field value in the FieldMessage class." << std::endl;
        return 0.0;
    }
}

FieldLineSegment FieldMessage::get(FieldLineName lineName) const {
    if (fieldLines.find(lineName) != fieldLines.end()) {
        return fieldLines.at(lineName);
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
//        std::cout << "Access undefined field line in the FieldMessage class." << std::endl;
        return {};
    }
}

FieldArc FieldMessage::get(FieldArcName arcName) const {
    if (fieldArcs.find(arcName) != fieldArcs.end()) {
        return fieldArcs.at(arcName);
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
//        std::cout << "Access undefined field arc in the FieldMessage class." << std::endl;
        return {};
    }
}

Vector2 FieldMessage::get(FieldVectorName vectorName) const {
    if (fieldVectors.find(vectorName) != fieldVectors.end()) {
        return fieldVectors.at(vectorName);
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
//        std::cout << "Access undefined field vector in the FieldMessage class." << std::endl;
        return {};
    }
}

std::unordered_map<FieldLineName, FieldLineSegment, std::hash<int>> FieldMessage::getField_lines() { return fieldLines; }
}  // namespace rtt