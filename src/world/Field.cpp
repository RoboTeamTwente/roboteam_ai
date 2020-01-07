//
// Created by Lukas Bos on 30/08/2019.
//

#include <include/roboteam_ai/world/Field.h>
namespace rtt {

Field Field::field = Field();
std::mutex Field::fieldMutex;

Field::Field(proto::SSL_GeometryFieldSize sslFieldSize) {
    initFieldLines(sslFieldSize);
    initFieldArcs(sslFieldSize);
    initFieldValues(sslFieldSize);
    initFieldVectors();
}

Field Field::get_field() {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return Field::field;
}

void Field::set_field(Field _field) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    Field::field = std::move(_field);
}

void Field::initFieldValues(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    fieldValues[FIELD_LENGTH] = mm_to_m(sslFieldSize.field_length());
    fieldValues[FIELD_WIDTH] = mm_to_m(sslFieldSize.field_width());
    fieldValues[GOAL_WIDTH] = mm_to_m(sslFieldSize.goal_width());
    fieldValues[GOAL_DEPTH] = mm_to_m(sslFieldSize.goal_depth());
    fieldValues[BOUNDARY_WIDTH] = mm_to_m(sslFieldSize.boundary_width());
    fieldValues[LEFTMOST_X] = -0.5 * fieldValues[FIELD_LENGTH].value();
    fieldValues[RIGHTMOST_X] = 0.5 * fieldValues[FIELD_LENGTH].value();
    fieldValues[BOTTOMMOST_Y] = -0.5 * fieldValues[FIELD_WIDTH].value();
    fieldValues[TOPMOST_Y] = 0.5 * fieldValues[FIELD_WIDTH].value();
    fieldValues[CENTER_Y] = 0.0;
}

void Field::initFieldLines(const proto::SSL_GeometryFieldSize &sslFieldSize) {
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

void Field::initFieldArcs(const proto::SSL_GeometryFieldSize &sslFieldSize) {
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

void Field::initFieldVectors() {
    fieldVectors[OUR_GOAL_CENTER] = Vector2(fieldValues[LEFTMOST_X].value(), fieldValues[CENTER_Y].value());
    fieldVectors[THEIR_GOAL_CENTER] = Vector2(fieldValues[RIGHTMOST_X].value(), fieldValues[CENTER_Y].value());

    Vector2 goalWidthAdjust = Vector2(0, fieldValues[GOAL_WIDTH].value() / 2);
    fieldVectors[OUR_BOTTOM_GOAL_SIDE] = fieldVectors[OUR_GOAL_CENTER].value() - goalWidthAdjust;
    fieldVectors[OUR_TOP_GOAL_SIDE] = fieldVectors[OUR_GOAL_CENTER].value() + goalWidthAdjust;
    fieldVectors[THEIR_BOTTOM_GOAL_SIDE] = fieldVectors[THEIR_GOAL_CENTER].value() - goalWidthAdjust;
    fieldVectors[THEIR_TOP_GOAL_SIDE] = fieldVectors[THEIR_GOAL_CENTER].value() + goalWidthAdjust;

    Vector2 lpl_begin = fieldLines[LEFT_PENALTY_LINE].value().begin;
    Vector2 lpl_end = fieldLines[LEFT_PENALTY_LINE].value().end;
    fieldVectors[LEFT_PENALTY_POINT] = lpl_begin + ((lpl_end - lpl_begin) * 0.5);

    Vector2 rpl_begin = fieldLines[RIGHT_PENALTY_LINE].value().begin;
    Vector2 rpl_end = fieldLines[RIGHT_PENALTY_LINE].value().end;
    fieldVectors[RIGHT_PENALTY_POINT] = rpl_begin + ((rpl_end - rpl_begin) * 0.5);
}

float Field::mm_to_m(float scalar) {
    return scalar / 1000;
}

Vector2 Field::mm_to_m(Vector2 vector) {
    return {vector.x / 1000, vector.y / 1000};
}

double Field::operator[](FieldValueName valueName) const {
    if (fieldValues[valueName]) {
        return fieldValues[valueName].value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. So the values are equal to 0.0 */
        std::cout << "Access undefined field value in the Field class." << std::endl;
        return 0.0;
    }
}

FieldLineSegment Field::operator[](FieldLineName lineName) const {
    if (fieldLines[lineName]) {
        return fieldLines[lineName].value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Access undefined field line in the Field class." << std::endl;
        return {};
    }
}

FieldArc Field::operator[](FieldArcName arcName) const {
    if (fieldArcs[arcName]) {
        return fieldArcs[arcName].value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Access undefined field arc in the Field class." << std::endl;
        return {};
    }
}

Vector2 Field::operator[](FieldVectorName vectorName) const {
    if (fieldVectors[vectorName]) {
        return fieldVectors[vectorName].value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Access undefined field vector in the Field class." << std::endl;
        return {};
    }
}

std::optional<FieldLineSegment>* Field::getField_lines(){
    return fieldLines;
}
}