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
            /*
            std::cout << newLine.name << std::endl;
            std::cout << newLine.begin.x << std::endl;
            std::cout << newLine.begin.y << std::endl;
            std::cout << newLine.end.x << std::endl;
            std::cout << newLine.end.y << std::endl;
            */
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
    return scalar / 1000;
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
    if (fieldLines.count(lineName) > 0) {
        return fieldLines.at(lineName);
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        return {};
    }
}

FieldArc FieldMessage::get(FieldArcName arcName) {
    if (fieldArcs.count(arcName) > 0) {
        return fieldArcs.at(arcName);
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        return {};
    }
}

std::vector<FieldLineSegment> FieldMessage::getField_lines(){
    std::vector<FieldLineSegment> allLines = {};
    for (auto &item : fieldLines) {
        allLines.push_back(item.second);
    }
    return allLines;
}

std::vector<FieldArc> FieldMessage::getField_arcs(){
    std::vector<FieldArc> allArcs = {};
    for (auto &item : fieldArcs) {
        allArcs.push_back(item.second);
    }
    return allArcs;
}

}