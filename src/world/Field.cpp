// Created by Lukas Bos on 30/08/2019.

#include "world/Field.h"

#include <roboteam_utils/Print.h>

#include "utilities/Settings.h"

namespace rtt::world {

Field::Field(proto::SSL_GeometryFieldSize sslFieldSize) {
    initFieldLines(sslFieldSize);
    initFieldArcs(sslFieldSize);
    initFieldValues(sslFieldSize);
    initFieldOthers();
    initFieldGrids();
}

void Field::initFieldValues(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    fieldLength = mm_to_m(sslFieldSize.field_length());
    fieldWidth = mm_to_m(sslFieldSize.field_width());
    goalWidth = mm_to_m(sslFieldSize.goal_width());
    goalDepth = mm_to_m(sslFieldSize.goal_depth());
    boundaryWidth = mm_to_m(sslFieldSize.boundary_width());
}

void Field::initFieldLines(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    // Used to convert field line name, in string format, to the corresponding FieldLineName enum value
    for (const proto::SSL_FieldLineSegment &line : sslFieldSize.field_lines()) {
        FieldLineSegment newLine;
        if (NAME_MAP.find(line.name()) != NAME_MAP.end()) {
            newLine.name = std::string(NAME_MAP[line.name()]);

            newLine.begin = mm_to_m({line.p1().x(), line.p1().y()});

            newLine.end = mm_to_m({line.p2().x(), line.p2().y()});
            mm_to_m(line.thickness());
            *(RELATED_FIELD_LINE[newLine.name]) = newLine;
            this->allFieldLines.push_back(newLine);
        }
    }
}

void Field::initFieldArcs(const proto::SSL_GeometryFieldSize &sslFieldSize) {
    for (const auto &arc : sslFieldSize.field_arcs()) {
        FieldArc newArc;
        if (NAME_MAP.find(arc.name()) != NAME_MAP.end()) {
            newArc.name = std::string(NAME_MAP[arc.name()]);
            newArc.center = mm_to_m({arc.center().x(), arc.center().y()});
            newArc.a1 = mm_to_m(arc.a1());
            newArc.a2 = mm_to_m(arc.a2());
            newArc.radius = mm_to_m(arc.radius());
            mm_to_m(arc.thickness());
            *(RELATED_FIELD_ARC[newArc.name]) = newArc;
        }
    }
}

void Field::initFieldGrids() {
    auto gridLength = getFieldLength() / numSegmentsX;
    auto gridWidth = getFieldWidth() / numSegmentsY;

    auto bottomX = getLeftmostX();
    auto middleX = getLeftmostX() + gridLength;
    auto topX = getLeftmostX() + gridLength * 2;

    auto leftY = getBottommostY() + gridWidth * 2;
    auto middleY = getBottommostY() + gridWidth;
    auto rightY = getBottommostY();

    backLeftGrid = Grid(bottomX, leftY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    backMidGrid = Grid(bottomX, middleY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    backRightGrid = Grid(bottomX, rightY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    middleLeftGrid = Grid(middleX, leftY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    middleMidGrid = Grid(middleX, middleY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    middleRightGrid = Grid(middleX, rightY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    frontLeftGrid = Grid(topX, leftY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    frontMidGrid = Grid(topX, middleY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
    frontRightGrid = Grid(topX, rightY, gridWidth, gridLength, numSegmentsX, numSegmentsY);
}

void Field::initFieldOthers() {
    // Initialize some additional field values
    leftmostX = -0.5 * fieldLength.value();
    rightmostX = 0.5 * fieldLength.value();
    bottommostY = -0.5 * fieldWidth.value();
    topmostY = 0.5 * fieldWidth.value();
    centerY = 0.0;
    leftPenaltyX = leftPenaltyLine.value().begin.x;
    rightPenaltyX = rightPenaltyLine.value().begin.x;

    // Initialize some additional field vectors
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

    leftPenaltyLineBottom = leftPenaltyLine->begin;
    leftPenaltyLineTop = leftPenaltyLine->end;
    rightPenaltyLineBottom = rightPenaltyLine->begin;
    rightPenaltyLineTop = rightPenaltyLine->end;

    bottomLeftCorner = Vector2(leftmostX.value(), bottommostY.value());
    topLeftCorner = Vector2(leftmostX.value(), topmostY.value());
    bottomRightCorner = Vector2(rightmostX.value(), bottommostY.value());
    topRightCorner = Vector2(rightmostX.value(), topmostY.value());

    topLeftOurDefenceArea = topLeftPenaltyStretch->begin;
    bottomLeftOurDefenceArea = bottomLeftPenaltyStretch->begin;
    topRightTheirDefenceArea = topRightPenaltyStretch->begin;
    bottomRightTheirDefenceArea = bottomRightPenaltyStretch->begin;
}

float Field::mm_to_m(float scalar) { return scalar / 1000; }

Vector2 Field::mm_to_m(const Vector2 &vector) { return {vector.x / 1000, vector.y / 1000}; }

double Field::getFieldWidth() const { return getFieldValue(fieldWidth); }

double Field::getFieldLength() const { return getFieldValue(fieldLength); }

double Field::getGoalWidth() const { return getFieldValue(goalWidth); }

double Field::getGoalDepth() const { return getFieldValue(goalDepth); }

double Field::getBoundaryWidth() const { return getFieldValue(boundaryWidth); }

double Field::getLeftmostX() const { return getFieldValue(leftmostX); }

double Field::getRightmostX() const { return getFieldValue(rightmostX); }

double Field::getBottommostY() const { return getFieldValue(bottommostY); }

double Field::getTopmostY() const { return getFieldValue(topmostY); }

double Field::getLeftPenaltyX() const { return getFieldValue(leftPenaltyX); }

double Field::getRightPenaltyX() const { return getFieldValue(rightPenaltyX); }

const FieldLineSegment &Field::getLeftLine() const { return getFieldLine(leftLine); }

const FieldLineSegment &Field::getRightLine() const { return getFieldLine(rightLine); }

const FieldLineSegment &Field::getLeftPenaltyLine() const { return getFieldLine(leftPenaltyLine); }

const FieldLineSegment &Field::getRightPenaltyLine() const { return getFieldLine(rightPenaltyLine); }

const Vector2 &Field::getOurGoalCenter() const { return getFieldVector(ourGoalCenter); }

const Vector2 &Field::getTheirGoalCenter() const { return getFieldVector(theirGoalCenter); }

const Vector2 &Field::getLeftPenaltyPoint() const { return getFieldVector(leftPenaltyPoint); }

const Vector2 &Field::getRightPenaltyPoint() const { return getFieldVector(rightPenaltyPoint); }

const Vector2 &Field::getOurBottomGoalSide() const { return getFieldVector(ourBottomGoalSide); }

const Vector2 &Field::getOurTopGoalSide() const { return getFieldVector(ourTopGoalSide); }

const Vector2 &Field::getTheirBottomGoalSide() const { return getFieldVector(theirBottomGoalSide); }

const Vector2 &Field::getTheirTopGoalSide() const { return getFieldVector(theirTopGoalSide); }

const Vector2 &Field::getLeftPenaltyLineTop() const { return getFieldVector(leftPenaltyLineTop); }

const Vector2 &Field::getLeftPenaltyLineBottom() const { return getFieldVector(leftPenaltyLineBottom); }

const Vector2 &Field::getRightPenaltyLineTop() const { return getFieldVector(rightPenaltyLineTop); }

const Vector2 &Field::getRightPenaltyLineBottom() const { return getFieldVector(rightPenaltyLineBottom); }

const FieldArc &Field::getCenterCircle() const { return getFieldArc(centerCircle); }

const Vector2 &Field::getBottomLeftCorner() const { return getFieldVector(bottomLeftCorner); }

const Vector2 &Field::getTopLeftCorner() const { return getFieldVector(topLeftCorner); }

const Vector2 &Field::getBottomRightCorner() const { return getFieldVector(bottomRightCorner); }

const Vector2 &Field::getTopRightCorner() const { return getFieldVector(topRightCorner); }

const Vector2 &Field::getTopLeftOurDefenceArea() const { return getFieldVector(topLeftOurDefenceArea); }

const Vector2 &Field::getBottomLeftOurDefenceArea() const { return getFieldVector(bottomLeftOurDefenceArea); }

const Vector2 &Field::getTopRightTheirDefenceArea() const { return getFieldVector(topRightTheirDefenceArea); }

const Vector2 &Field::getBottomRightTheirDefenceArea() const { return getFieldVector(bottomRightTheirDefenceArea); }

const Grid &Field::getBackLeftGrid() const { return getFieldGrid(backLeftGrid); }

const Grid &Field::getBackMidGrid() const { return getFieldGrid(backMidGrid); }

const Grid &Field::getBackRightGrid() const { return getFieldGrid(backRightGrid); }

const Grid &Field::getMiddleLeftGrid() const { return getFieldGrid(middleLeftGrid); }

const Grid &Field::getMiddleMidGrid() const { return getFieldGrid(middleMidGrid); }

const Grid &Field::getMiddleRightGrid() const { return getFieldGrid(middleRightGrid); }

const Grid &Field::getFrontLeftGrid() const { return getFieldGrid(frontLeftGrid); }

const Grid &Field::getFrontMidGrid() const { return getFieldGrid(frontMidGrid); }

const Grid &Field::getFrontRightGrid() const { return getFieldGrid(frontRightGrid); }

double Field::getFieldValue(const std::optional<double> &fieldValue) const {
    if (fieldValue) {
        return fieldValue.value();
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. So the values are equal to 0.0 */
        RTT_WARNING("Access undefined field value in the Field class (world might not be turned on?).")
        return 0.0;
    }
}

const FieldLineSegment &Field::getFieldLine(const std::optional<FieldLineSegment> &fieldLine) const {
    if (fieldLine) {
        return fieldLine.value();
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        RTT_WARNING("Access undefined field line in the Field class (world might not be turned on?).")

        static FieldLineSegment standard = {};
        return standard;
    }
}

const Vector2 &Field::getFieldVector(const std::optional<Vector2> &fieldVector) const {
    if (fieldVector) {
        return fieldVector.value();
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        RTT_WARNING("Access undefined field vector in the Field class (world might not be turned on?).")

        static Vector2 standard = {};
        return standard;
    }
}

const FieldArc &Field::getFieldArc(const std::optional<FieldArc> &fieldArc) const {
    if (fieldArc) {
        return fieldArc.value();
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        RTT_WARNING("Access undefined field arc in the Field class (world might not be turned on?).")

        static FieldArc standard = {};
        return standard;
    }
}

const Grid &Field::getFieldGrid(const std::optional<Grid> &fieldGrid) const {
    if (fieldGrid) {
        return fieldGrid.value();
    } else {
        /* This clause is needed, because the default constructor could have been called. In which case the variables
        have not been assigned a value. */
        RTT_WARNING("Access undefined grid in the Field class (world might not be turned on?).")

        static Grid standard = Grid(0, 0, 0, 0, 0, 0);
        return standard;
    }
}

const std::vector<FieldLineSegment> &Field::getFieldLines() const { return allFieldLines; }

Field Field::createTestField() {
    double fieldWidth = 9;
    double fieldLength = 12;
    double goalWidth = 1.2000000476837158;
    double goalDepth = 0.20000000298023224;
    double boundaryWidth = 0.30000001192092896;
    FieldLineSegment topLine = {{-5.9950000000000001, 4.4950000000000001}, {5.9950000000000001, 4.4950000000000001}, "top_line"};
    FieldLineSegment bottomLine = {{-5.9950000000000001, -4.4950000000000001}, {5.9950000000000001, -4.4950000000000001}, "bottom_line"};
    FieldLineSegment leftLine = {{-5.9900000000000002, -4.4950000000000001}, {-5.9900000000000002, 4.4950000000000001}, "left_line"};
    FieldLineSegment rightLine = {{5.9900000000000002, -4.4950000000000001}, {5.9900000000000002, 4.4950000000000001}, "right_line"};
    FieldLineSegment halfLine = {{0, -4.4950000000000001}, {0, 4.4950000000000001}, "half_line"};
    FieldLineSegment centerLine = {{-5.9900000000000002, 0}, {5.9900000000000002, 0}, "center_line"};
    FieldLineSegment leftPenaltyLine = {{-4.7949999999999999, -1.2}, {-4.7949999999999999, 1.2}, "left_penalty_line"};
    FieldLineSegment rightPenaltyLine = {{4.7949999999999999, -1.2}, {4.7949999999999999, 1.2}, "right_penalty_line"};
    FieldLineSegment topLeftPenaltyStretch = {{-5.9900000000000002, 1.2}, {-4.79, 1.2}, "top_left_penalty_stretch"};
    FieldLineSegment bottomLeftPenaltyStretch = {{-5.9900000000000002, -1.2}, {-4.79, -1.2}, "bottom_left_penalty_stretch"};
    FieldLineSegment topRightPenaltyStretch = {{5.9900000000000002, 1.2}, {4.79, 1.2}, "top_right_penalty_stretch"};
    FieldLineSegment bottomRightPenaltyStretch = {{5.9900000000000002, -1.2}, {4.79, -1.2}, "bottom_right_penalty_stretch"};
    FieldArc centerCircle = {{0, 0}, 0.495000005, 0, 0.00628318544, "center_circle"};
    return Field(fieldWidth, fieldLength, goalWidth, goalDepth, boundaryWidth, topLine, bottomLine, leftLine, rightLine, halfLine, centerLine, leftPenaltyLine, rightPenaltyLine,
                 topLeftPenaltyStretch, bottomLeftPenaltyStretch, topRightPenaltyStretch, bottomRightPenaltyStretch, centerCircle);
}

Field::Field(double fieldWidth, double fieldLength, double goalWidth, double goalDepth, double boundaryWidth, FieldLineSegment &topLine, FieldLineSegment &bottomLine,
             FieldLineSegment &leftLine, FieldLineSegment &rightLine, FieldLineSegment &halfLine, FieldLineSegment &centerLine, FieldLineSegment &leftPenaltyLine,
             FieldLineSegment &rightPenaltyLine, FieldLineSegment &topLeftPenaltyStretch, FieldLineSegment &bottomLeftPenaltyStretch, FieldLineSegment &topRightPenaltyStretch,
             FieldLineSegment &bottomRightPenaltyStretch, FieldArc &centerCircle) {
    this->fieldWidth = fieldWidth;
    this->fieldLength = fieldLength;
    this->goalWidth = goalWidth;
    this->goalDepth = goalDepth;
    this->boundaryWidth = boundaryWidth;
    this->topLine = topLine;
    this->bottomLine = bottomLine;
    this->leftLine = leftLine;
    this->rightLine = rightLine;
    this->halfLine = halfLine;
    this->centerLine = centerLine;
    this->leftPenaltyLine = leftPenaltyLine;
    this->rightPenaltyLine = rightPenaltyLine;
    this->topLeftPenaltyStretch = topLeftPenaltyStretch;
    this->bottomLeftPenaltyStretch = bottomLeftPenaltyStretch;
    this->topRightPenaltyStretch = topRightPenaltyStretch;
    this->bottomRightPenaltyStretch = bottomRightPenaltyStretch;
    this->centerCircle = centerCircle;

    initFieldOthers();
}

Field &Field::operator=(const Field &old) noexcept {
    if (this == &old) {
        return *this;
    }
    // this->NAME_MAP already properly set
    // this->RELATED_FIELD_LINE already properly set.
    // this->RELATED_FIELD_ARC already properly set
    /**
     * If only padding in C++ was guaranteed and i could just do
     * auto maps_size = sizeof(std::decay_t<decltype(this->NAME_MAP)>) * 3;
     * auto copy_bytes = sizeof(std::decay_t<*this>) - maps_size;
     * std::memcpy((char*)this + maps_size, (const char*)&old + maps_size, copy_bytes);
     * I cri, haico pls never make something like an internal map with pointers to members of `this` again
     *
     * or @bjarne please add reflection ;) ;) ;) <3
     */
    allFieldLines = old.allFieldLines;
    fieldWidth = old.fieldWidth;
    fieldLength = old.fieldLength;
    goalWidth = old.goalWidth;
    goalDepth = old.goalDepth;
    boundaryWidth = old.boundaryWidth;
    centerY = old.centerY;
    leftmostX = old.leftmostX;
    rightmostX = old.rightmostX;
    bottommostY = old.bottommostY;
    topmostY = old.topmostY;
    leftPenaltyX = old.leftPenaltyX;
    rightPenaltyX = old.rightPenaltyX;
    penaltyTopY = old.penaltyTopY;
    penaltyBottomY = old.penaltyBottomY;
    topLine = old.topLine;
    bottomLine = old.bottomLine;
    leftLine = old.leftLine;
    rightLine = old.rightLine;
    halfLine = old.halfLine;
    centerLine = old.centerLine;
    leftPenaltyLine = old.leftPenaltyLine;
    rightPenaltyLine = old.rightPenaltyLine;
    topLeftPenaltyStretch = old.topLeftPenaltyStretch;
    bottomLeftPenaltyStretch = old.bottomLeftPenaltyStretch;
    topRightPenaltyStretch = old.topRightPenaltyStretch;
    bottomRightPenaltyStretch = old.bottomRightPenaltyStretch;
    ourGoalCenter = old.ourGoalCenter;
    theirGoalCenter = old.theirGoalCenter;
    leftPenaltyPoint = old.leftPenaltyPoint;
    rightPenaltyPoint = old.rightPenaltyPoint;
    ourBottomGoalSide = old.ourBottomGoalSide;
    ourTopGoalSide = old.ourTopGoalSide;
    theirBottomGoalSide = old.theirBottomGoalSide;
    theirTopGoalSide = old.theirTopGoalSide;
    leftPenaltyLineTop = old.leftPenaltyLineTop;
    leftPenaltyLineBottom = old.leftPenaltyLineBottom;
    rightPenaltyLineTop = old.rightPenaltyLineTop;
    rightPenaltyLineBottom = old.rightPenaltyLineBottom;
    bottomLeftCorner = old.bottomLeftCorner;
    topLeftCorner = old.topLeftCorner;
    bottomRightCorner = old.bottomRightCorner;
    topRightCorner = old.topRightCorner;
    centerCircle = old.centerCircle;
    topLeftOurDefenceArea = old.topLeftOurDefenceArea;
    bottomLeftOurDefenceArea = old.bottomLeftOurDefenceArea;
    topRightTheirDefenceArea = old.topRightTheirDefenceArea;
    bottomRightTheirDefenceArea = old.bottomRightTheirDefenceArea;
    backLeftGrid = old.backLeftGrid;
    backMidGrid = old.backMidGrid;
    backRightGrid = old.backRightGrid;
    middleLeftGrid = old.middleLeftGrid;
    middleMidGrid = old.middleMidGrid;
    middleRightGrid = old.middleRightGrid;
    frontLeftGrid = old.frontLeftGrid;
    frontMidGrid = old.frontMidGrid;
    frontRightGrid = old.frontRightGrid;
    return *this;
}

Field &Field::operator=(Field &&old) noexcept {
    NAME_MAP = std::move(old.NAME_MAP);
    *this = old;
    return *this;
}

Field::Field(Field &&old) noexcept {
    NAME_MAP = std::move(old.NAME_MAP);
    *this = old;
}

Field::Field(Field const &old) noexcept { *this = old; }

}  // namespace rtt::world
