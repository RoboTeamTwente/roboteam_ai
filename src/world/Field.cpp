//
// Created by Lukas Bos on 30/08/2019.
//

#include <include/roboteam_ai/world/Field.h>
namespace rtt {

Field::Field(proto::SSL_GeometryFieldSize sslFieldSize) {
    initFieldLines(sslFieldSize);
    initFieldArcs(sslFieldSize);
    initFieldValues(sslFieldSize);
    initFieldVectors();
}

void Field::initFieldValues(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    fieldLength = mm_to_m(sslFieldSize.field_length());
    fieldWidth = mm_to_m(sslFieldSize.field_width());
    goalWidth = mm_to_m(sslFieldSize.goal_width());
    goalDepth = mm_to_m(sslFieldSize.goal_depth());
    boundaryWidth = mm_to_m(sslFieldSize.boundary_width());
    leftmostX = -0.5 * fieldLength.value();
    rightmostX = 0.5 * fieldLength.value();
    bottommostY = -0.5 * fieldWidth.value();
    topmostY = 0.5 * fieldWidth.value();
    centerY = 0.0;
}

void Field::initFieldLines(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    // Used to convert field line name, in string format, to the corresponding FieldLineName enum value
    for (const proto::SSL_FieldLineSegment &line : sslFieldSize.field_lines()) {
        FieldLineSegment newLine;
        if (NAME_MAP.find(line.name()) != NAME_MAP.end()) {
            newLine.name = std::string(NAME_MAP[line.name()]);
            newLine.begin = mm_to_m(line.p1());
            newLine.end = mm_to_m(line.p2());
            newLine.thickness = mm_to_m(line.thickness());
            *(RELATED_FIELD_LINE[newLine.name]) = newLine;
            allFieldLines.push_back(newLine);
        }
    }
}

void Field::initFieldArcs(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    for (const proto::SSL_FieldCicularArc &arc : sslFieldSize.field_arcs()) {
        FieldArc newArc;
        if (NAME_MAP.find(arc.name()) != NAME_MAP.end()) {
            newArc.name = std::string(NAME_MAP[arc.name()]);
            newArc.center = mm_to_m(arc.center());
            newArc.a1 = mm_to_m(arc.a1());
            newArc.a2 = mm_to_m(arc.a2());
            newArc.radius = mm_to_m(arc.radius());
            newArc.thickness = mm_to_m(arc.thickness());
            *(RELATED_FIELD_ARC[newArc.name]) = newArc;
        }
    }
}

void Field::initFieldVectors() {
    ourGoalCenter = Vector2(leftmostX.value(), centerY.value());
    theirGoalCenter = Vector2(rightmostX.value(), centerY.value());

    Vector2 goalWidthAdjust = Vector2(0, goalWidth.value() / 2);
    ourBottomGoalSide = ourGoalCenter.value() - goalWidthAdjust;
    ourTopGoalSide = ourGoalCenter.value() + goalWidthAdjust;
    theirBottomGoalSide = theirGoalCenter.value() - goalWidthAdjust;
    theirTopGoalSide = theirGoalCenter.value() + goalWidthAdjust;

    Vector2 lpl_begin = leftPenaltyLine.value().begin;
    Vector2 lpl_end = leftPenaltyLine.value().end;
    leftPenaltyPoint = lpl_begin + ((lpl_end - lpl_begin) * 0.5);

    Vector2 rpl_begin = rightPenaltyLine.value().begin;
    Vector2 rpl_end = rightPenaltyLine.value().end;
    rightPenaltyPoint = rpl_begin + ((rpl_end - rpl_begin) * 0.5);
}

float Field::mm_to_m(float scalar) {
    return scalar / 1000;
}

Vector2 Field::mm_to_m(Vector2 vector) {
    return {vector.x / 1000, vector.y / 1000};
}

double Field::getFieldWidth() const {
    return getFieldValue(fieldWidth);
}

double Field::getFieldLength() const {
    return getFieldValue(fieldLength);
}

double Field::getGoalWidth() const {
    return getFieldValue(goalWidth);
}

double Field::getGoalDepth() const {
    return getFieldValue(goalDepth);
}

double Field::getBoundaryWidth() const {
    return getFieldValue(boundaryWidth);
}

double Field::getCenterY() const {
    return getFieldValue(centerY);
}

double Field::getLeftmostX() const {
    return getFieldValue(leftmostX);
}

double Field::getRightmostX() const {
    return getFieldValue(rightmostX);
}

double Field::getBottommostY() const {
    return getFieldValue(bottommostY);
}

double Field::getTopmostY() const {
    return getFieldValue(topmostY);
}

const FieldLineSegment &Field::getTopLine() const {
    return getFieldLine(topLine);
}

const FieldLineSegment &Field::getBottomLine() const {
    return getFieldLine(bottomLine);
}

const FieldLineSegment &Field::getLeftLine() const {
    return getFieldLine(leftLine);
}

const FieldLineSegment &Field::getRightLine() const {
    return getFieldLine(rightLine);
}

const FieldLineSegment &Field::getHalfLine() const {
    return getFieldLine(halfLine);
}

const FieldLineSegment &Field::getCenterLine() const {
    return getFieldLine(centerLine);
}

const FieldLineSegment &Field::getLeftPenaltyLine() const {
    return getFieldLine(leftPenaltyLine);
}

const FieldLineSegment &Field::getRightPenaltyLine() const {
    return getFieldLine(rightPenaltyLine);
}

const FieldLineSegment &Field::getTopLeftPenaltyStretch() const {
    return getFieldLine(topLeftPenaltyStretch);
}

const FieldLineSegment &Field::getBottomLeftPenaltyStretch() const {
    return getFieldLine(bottomLeftPenaltyStretch);
}

const FieldLineSegment &Field::getTopRightPenaltyStretch() const {
    return getFieldLine(topRightPenaltyStretch);
}

const FieldLineSegment &Field::getBottomRightPenaltyStretch() const {
    return getFieldLine(bottomRightPenaltyStretch);
}

const Vector2 &Field::getOurGoalCenter() const {
    return getFieldVector(ourGoalCenter);
}

const Vector2 &Field::getTheirGoalCenter() const {
    return getFieldVector(theirGoalCenter);
}

const Vector2 &Field::getLeftPenaltyPoint() const {
    return getFieldVector(leftPenaltyPoint);
}

const Vector2 &Field::getRightPenaltyPoint() const {
    return getFieldVector(rightPenaltyPoint);
}

const Vector2 &Field::getOurBottomGoalSide() const {
    return getFieldVector(ourBottomGoalSide);
}

const Vector2 &Field::getOurTopGoalSide() const {
    return getFieldVector(ourTopGoalSide);
}

const Vector2 &Field::getTheirBottomGoalSide() const {
    return getFieldVector(theirBottomGoalSide);
}

const Vector2 &Field::getTheirTopGoalSide() const {
    return getFieldVector(theirTopGoalSide);
}

const FieldArc &Field::getCenterCircle() const {
    return getFieldArc(centerCircle);
}

double Field::getFieldValue(const std::optional<double> &fieldValue) const {
    if (fieldValue) {
        return fieldValue.value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. So the values are equal to 0.0 */
        std::cout << "Warning: access undefined field value in the Field class (world might not be turned on?)." << std::endl;
        return 0.0;
    }
}

const FieldLineSegment &Field::getFieldLine(const std::optional<FieldLineSegment> &fieldLine) const {
    if (fieldLine) {
        return fieldLine.value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Warning: access undefined field line in the Field class (world might not be turned on?)." << std::endl;

        static FieldLineSegment standard = {};
        return standard;
    }
}

const Vector2 &Field::getFieldVector(const std::optional<Vector2> &fieldVector) const {
    if (fieldVector) {
        return fieldVector.value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Warning: access undefined field vector in the Field class (world might not be turned on?)." << std::endl;

        static Vector2 standard = {};
        return standard;
    }
}

const FieldArc &Field::getFieldArc(const std::optional<FieldArc> &fieldArc) const {
    if (fieldArc) {
        return fieldArc.value();
    }
    else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        std::cout << "Warning: access undefined field arc in the Field class (world might not be turned on?)." << std::endl;

        static FieldArc standard = {};
        return standard;
    }
}

const std::vector<FieldLineSegment> &Field::getFieldLines() const {
    return allFieldLines;
}

}