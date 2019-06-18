//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <ros/node_handle.h>
#include <roboteam_ai/src/coach/PassCoach.h>
#include "widget.h"
#include "roboteam_ai/src/interface/api/Input.h"
#include "roboteam_ai/src/interface/api/Output.h"
#include "roboteam_ai/src/analysis/GameAnalyzer.h"

namespace rtt {
namespace ai {
namespace interface {

Visualizer::Visualizer(QWidget* parent)
        :QWidget(parent) { }

/// The update loop of the field widget. Invoked by widget->update();
void Visualizer::paintEvent(QPaintEvent* event) {
    QPainter painter(this);

    calculateFieldSizeFactor();
    if (rtt::ai::world::world->weHaveRobots()) {
        drawBackground(painter);
        drawFieldHints(painter);
        drawFieldLines(painter);
        drawRobots(painter);
        drawBall(painter);

        // draw the drawings from the input
        auto drawings = Input::getDrawings();
        for (auto const &drawing : drawings) {
            if (! drawing.points.empty()) {

                bool shouldShow = false;
                for (auto const &toggle : Toggles::toggles) {
                    if (drawing.visual == toggle.visual) {
                        shouldShow = shouldVisualize(toggle, drawing.robotId);
                    }
                }
                if (shouldShow) {
                    switch (drawing.method) {
                    case Drawing::DOTS: {
                        painter.setPen(Qt::NoPen);
                        painter.setBrush(drawing.color);
                        drawPoints(painter, drawing.points, drawing.width, drawing.height);
                    }
                        break;
                    case Drawing::CIRCLES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawPoints(painter, drawing.points, drawing.width, drawing.height);
                    }
                        break;
                    case Drawing::LINES_CONNECTED: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawLines(painter, drawing.points);
                    }
                        break;
                    case Drawing::CROSSES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawCrosses(painter, drawing.points, drawing.width, drawing.height);
                    }
                        break;
                    case Drawing::PLUSSES: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawPlusses(painter, drawing.points, drawing.width, drawing.height);
                    }
                    case Drawing::ARROWS: {
                        painter.setPen(drawing.color);
                        painter.setBrush(Qt::transparent);
                        drawArrows(painter, drawing.points, drawing.width, drawing.height, drawing.strokeWidth==1);
                    }
                    }
                }
            }
        }
        Input::clearDrawings();

        if (showBallPlacementMarker) drawBallPlacementTarget(painter);
    }
    else {
        painter.drawText(24, 24, "Waiting for incoming World State");
    }
}

bool Visualizer::shouldVisualize(Toggle toggle, int robotId) {
    switch (toggle.showType) {
    default:return false;
    case GENERAL: {
        switch (toggle.generalShowType) {
        default:return false;
        case OFF:return false;
        case ON:return true;
        }
        break;
    }
    case ROBOT: {
        switch (toggle.robotShowType) {
        default:return false;
        case NO_ROBOTS:return false;
        case SELECTED_ROBOTS:return robotIsSelected(robotId);
        case ALL_ROBOTS:return true;
        }
    }
    }
}

/// Calculates the factor variable which is used for mapping field coordinates with screen coordinates.
void Visualizer::calculateFieldSizeFactor() {
    roboteam_msgs::GeometryFieldSize field = rtt::ai::world::field->get_field();
    fieldmargin = static_cast<int>(Constants::WINDOW_FIELD_MARGIN() + field.boundary_width);
    float widthFactor = this->size().width()/field.field_length - (2*fieldmargin);
    float heightFactor = this->size().height()/field.field_width - (2*fieldmargin);
    factor = std::min(widthFactor, heightFactor);
}

/// draws background of the field
void Visualizer::drawBackground(QPainter &painter) {
    painter.setBrush(Constants::FIELD_COLOR());
    painter.drawRect(- 10, - 10, this->size().width() + 10, this->size().height() + 10);
}

// draws the field lines
void Visualizer::drawFieldLines(QPainter &painter) {
    painter.setPen(Constants::FIELD_LINE_COLOR());
    painter.setBrush(Qt::transparent);
    // draw lines
    for (auto &line : rtt::ai::world::field->get_field().field_lines) {
        rtt::Vector2 start = toScreenPosition(line.begin);
        rtt::Vector2 end = toScreenPosition(line.end);
        painter.drawLine(start.x, start.y, end.x, end.y);
    }

    // draw the circle in the middle
    auto centercircle = rtt::ai::world::field->get_field().center_circle;
    Vector2 screenPos = toScreenPosition({centercircle.center.x, centercircle.center.y});
    painter.drawEllipse(QPointF(screenPos.x, screenPos.y), centercircle.radius*factor, centercircle.radius*factor);



        QPen pen;
        pen.setWidth(3);

        ros::NodeHandle nh;
        std::string ourColorParam;
        nh.getParam("our_color", ourColorParam);

        // update the we are yellow
        bool weAreYellow = ourColorParam == "yellow";

        // draw the hint for us
        auto usGoalLine = world::field->getGoalSides(true);
        Vector2 ourLineUpper = {usGoalLine.first.x, usGoalLine.first.y};
        Vector2 ourLineLower = {usGoalLine.second.x, usGoalLine.second.y};
        ourLineUpper = toScreenPosition(ourLineUpper);
        ourLineLower = toScreenPosition(ourLineLower);

        auto color = weAreYellow ? QColor(255, 255, 0, 255) : QColor(80, 80, 255, 255);
        pen.setBrush(color);
        pen.setColor(color);
        painter.setPen(pen);
        painter.drawLine(ourLineUpper.x, ourLineUpper.y, ourLineLower.x, ourLineLower.y);


        auto theirGoalLine = world::field->getGoalSides(false);
        Vector2 theirLineUpper = {theirGoalLine.first.x, theirGoalLine.first.y};
        Vector2 theirLineLower = {theirGoalLine.second.x, theirGoalLine.second.y};
        theirLineUpper = toScreenPosition(theirLineUpper);
        theirLineLower = toScreenPosition(theirLineLower);

        color = weAreYellow ? QColor(80, 80, 255, 255) : QColor(255, 255, 0, 255);
        pen.setBrush(color);
        pen.setColor(color);
        painter.setPen(pen);
        painter.drawLine(theirLineUpper.x, theirLineUpper.y, theirLineLower.x, theirLineLower.y);


    }

void Visualizer::drawFieldHints(QPainter &painter) {
    QPen pen;

    // draw the position where robots would be for timeout
    int inv = rtt::ai::interface::Output::isTimeOutAtTop() ? 1 : - 1;
    int lineY = (rtt::ai::world::field->get_field().field_width/2 + 1)*inv;

    pen.setBrush(Qt::gray);
    pen.setColor(Qt::gray);
    painter.setPen(pen);

    auto lineStart = toScreenPosition(Vector2(world::field->get_our_goal_center().x, lineY));
    auto lineEnd = toScreenPosition(Vector2(0, lineY));

    painter.drawLine(lineStart.x, lineStart.y, lineEnd.x, lineEnd.y);

}

// draw the ball on the screen
void Visualizer::drawBall(QPainter &painter) {
    auto ball = world::world->getBall();
    if (! (ball && world::Ball::exists && ball->pos.isNotNaN())) return;

    rtt::Vector2 ballPosition = toScreenPosition(ball->pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);

    if (! ball->visible) {
        painter.setBrush(Qt::red); // fill
    }
    else {
        painter.setBrush(Constants::BALL_COLOR()); // fill
    }
    painter.setBrush(Constants::BALL_COLOR()); // fill

    // draw a see-through gradient around the ball to make it more visible
    painter.setPen(Qt::NoPen); // stroke
    painter.setOpacity(0.5);
    painter.drawEllipse(qballPosition, Constants::BALL_DRAWING_SIZE(), Constants::BALL_DRAWING_SIZE());
    painter.setOpacity(1);
    int ballSize = Constants::BALL_RADIUS()*2*factor;
    painter.drawEllipse(qballPosition, ballSize, ballSize);

}

// draw the robots
void Visualizer::drawRobots(QPainter &painter) {

    // draw us
    for (auto &robot : rtt::ai::world::world->getUs()) {
        drawRobot(painter, *robot, true);
    }

    // draw them
    for (auto &robot : rtt::ai::world::world->getThem()) {
        drawRobot(painter, *robot, false);
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toScreenPosition(rtt::Vector2 fieldPos) {
    int inv = fieldInversed ? - 1 : 1;
    auto x = (fieldPos.x*factor*inv) + static_cast<float>(this->size().width()/2 + fieldmargin);
    auto y = (fieldPos.y*factor*- 1*inv) + static_cast<float>(this->size().height()/2 + fieldmargin);
    return Vector2(x, y);
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toFieldPosition(rtt::Vector2 screenPos) {
    int inv = fieldInversed ? - 1 : 1;
    auto x = (screenPos.x - fieldmargin - static_cast<float>(this->size().width()/2))/factor;
    auto y = ((screenPos.y - fieldmargin - static_cast<float>(this->size().height()/2))/factor)*- 1;
    return Vector2(x, y)*inv;
}

// draw a single robot
void Visualizer::drawRobot(QPainter &painter, Robot robot, bool ourTeam) {
    Vector2 robotpos = toScreenPosition(robot.pos);
    QPointF qrobotPosition(robotpos.x, robotpos.y);

    // we check the ros param our_color every time. This is because the referee can switch colors at any moment.
    ros::NodeHandle nh;
    std::string ourColorParam, newParam;
    nh.getParam("our_color", ourColorParam);

    // update the we are yellow
    bool weAreYellow = ourColorParam == "yellow";

    QColor robotColor;
    if (ourTeam) {
        // our robots have our_color
        robotColor = weAreYellow ? Constants::ROBOT_COLOR_YELLOW() : Constants::ROBOT_COLOR_BLUE();
    }
    else {
        // the enemy robot should have the inverse of our_color
        robotColor = weAreYellow ? Constants::ROBOT_COLOR_BLUE() : Constants::ROBOT_COLOR_YELLOW();
    }

    if (ourTeam && robot.id == robotDealer::RobotDealer::getKeeperID()) {
        robotColor = QColor(255, 255, 255);
    }

    if (showAngles) {
        Vector2 angle = toScreenPosition({robot.pos.x + cos(robot.angle)/3, robot.pos.y + sin(robot.angle)/3});
        QPen pen;
        pen.setWidth(2);
        pen.setBrush(robotColor);
        painter.setPen(pen);
        painter.drawLine(robotpos.x, robotpos.y, angle.x, angle.y);
    }

    if (showVelocities) {
        Vector2 vel = toScreenPosition({robot.pos.x + robot.vel.x, robot.pos.y + robot.vel.y});
        painter.setPen(Qt::white);
        painter.drawLine(robotpos.x, robotpos.y, vel.x, vel.y);
    }

    if (showTacticColors && ourTeam) {
        drawTacticColorForRobot(painter, robot);
    }

    int ypos = robotpos.y;
    if (showTactics && ourTeam) {
        painter.setPen(Constants::TEXT_COLOR());
        painter.drawText(robotpos.x, ypos += 20, QString::fromStdString(getTacticNameForRobot(robot)));
    }

    if (showRoles && ourTeam) {
        painter.setPen(Constants::TEXT_COLOR());
        painter.drawText(robotpos.x, ypos += 20, QString::fromStdString(getRoleNameForRobot(robot)));
    }

    if (showRobotInvalids && ourTeam) {
        painter.setPen(Qt::red);
        std::string text;
        if (!robot.hasWorkingGeneva()) {
            text += "GV ";
        }
        if (!robot.hasWorkingDribbler()) {
            text += "DR ";
        }
        if (!robot.hasWorkingBallSensor()) {
            text += "BS ";
        }
        if (robot.isBatteryLow()) {
            text += "BATTERY LOW";
        }
        painter.drawText(robotpos.x, ypos += 20, QString::fromStdString(text));
    }

    // draw the robots
    QColor color = (robotIsSelected(robot) && ourTeam) ? Constants::SELECTED_ROBOT_COLOR() : robotColor;
    painter.setBrush(color);
    painter.setPen(Qt::transparent);

    if (ourTeam) {
        std::map<int, double> genevaToAngle;
        genevaToAngle[1] = - 20.0;
        genevaToAngle[2] = - 10.0;
        genevaToAngle[3] = 0.0;
        genevaToAngle[4] = 10.0;
        genevaToAngle[5] = 20.0;

        auto genevaAngle = robot.angle + toRadians(genevaToAngle[robot.getGenevaState()]);

        // draw the angle of the geneva
        Vector2 angle = toScreenPosition({robot.pos.x + cos(genevaAngle)/4, robot.pos.y + sin(genevaAngle)/4});
        QPen pen;
        pen.setWidth(2);
        pen.setColor(Qt::red);
        painter.setPen(pen);

        painter.setBrush(Qt::red);
        painter.drawLine(robotpos.x, robotpos.y, angle.x, angle.y);
    }

    painter.setBrush(color);
    painter.setPen(Qt::transparent);

    painter.setOpacity(1);

    int robotDrawSize = std::max(Constants::ROBOT_RADIUS()*factor*2, (double) Constants::ROBOT_DRAWING_SIZE());

    // draw the shape of the robot with the right angle
    QPainterPath rectPath;
    rectPath.moveTo(0, - robotDrawSize/2.0);
    rectPath.arcTo(- robotDrawSize/2.0, - robotDrawSize/2.0, robotDrawSize, robotDrawSize, 90, 270);
    rectPath.closeSubpath();

    painter.translate(robotpos.x, robotpos.y); // move center of coordinates to the center of robot

    if (fieldInversed) {
        painter.rotate(- toDegrees(robot.angle) + 45 + 180); // rotate around the center of robot
    }
    else {
        painter.rotate(- toDegrees(robot.angle) + 45); // rotate around the center of robot
    }
    painter.drawPath(rectPath);
    painter.resetTransform(); // reset the translation and rotation


    // draw the id in it
    painter.setPen(Qt::black);
    painter.setFont(QFont("ubuntu", 9)); //22 is a number which you have to change
    painter.drawText(robotpos.x - 3, robotpos.y + 5, QString::fromStdString(std::to_string(robot.id)));
    painter.setFont(QFont("ubuntu", 11)); //22 is a number which you have to change

}

// Handle mousePressEvents
void Visualizer::mousePressEvent(QMouseEvent* event) {
    Vector2 pos;
    pos.x = event->pos().x();
    pos.y = event->pos().y();

    if (event->button() == Qt::LeftButton) {
        for (auto &robot : rtt::ai::world::world->getWorld().us) {
            if (pos.dist(toScreenPosition(robot->pos)) < 10) {
                this->toggleSelectedRobot(robot->id);
            }
        }
    }
    else if (event->button() == Qt::RightButton) {
        Output::setMarkerPosition(toFieldPosition(pos));

    }
}

void Visualizer::drawTacticColorForRobot(QPainter &painter, Robot robot) {
    Vector2 robotpos = toScreenPosition(robot.pos);
    QPointF qrobotPosition(robotpos.x, robotpos.y);
    std::string tacticName = getTacticNameForRobot(robot);
    bool tacticExists = false;
    QColor c;
    for (auto tac : tacticColors) {
        if (tac.first == tacticName) {
            c = tac.second;
            tacticExists = true;
            break;
        }
    }

    if (! tacticExists) {
        QColor newColor = Constants::TACTIC_COLORS().at(tacticCount);
        tacticCount = (tacticCount + 1)%Constants::TACTIC_COLORS().size();
        tacticColors.push_back({tacticName, newColor});
        c = newColor;
    }

    painter.setPen(Qt::transparent);
    painter.setBrush(c);
    painter.drawEllipse(qrobotPosition, Constants::TACTIC_COLOR_DRAWING_SIZE(), Constants::TACTIC_COLOR_DRAWING_SIZE());
}

std::string Visualizer::getTacticNameForRobot(Robot robot) {
    return robotDealer::RobotDealer::getTacticNameForId(robot.id);
}

std::string Visualizer::getRoleNameForRobot(Robot robot) {
    return robotDealer::RobotDealer::getRoleNameForId(robot.id);
}

void Visualizer::setShowRoles(bool showRoles) {
    this->showRoles = showRoles;
}

void Visualizer::setShowTactics(bool showTactics) {
    Visualizer::showTactics = showTactics;
}

void Visualizer::setShowTacticColors(bool showTacticColors) {
    Visualizer::showTacticColors = showTacticColors;
}

const std::vector<rtt::ai::world::Robot> &Visualizer::getSelectedRobots() const {
    return selectedRobots;
}

void Visualizer::setShowAngles(bool showAngles) {
    Visualizer::showAngles = showAngles;
}

void Visualizer::setShowVelocities(bool showVelocities) {
    Visualizer::showVelocities = showVelocities;
}

void Visualizer::setShowRobotInvalids(bool show) {
    Visualizer::showRobotInvalids = show;
}

void Visualizer::toggleSelectedRobot(int robotId) {
    bool robotWasAlreadySelected = false;

    for (int i = 0; i < static_cast<int>(selectedRobots.size()); i ++) {
        if (static_cast<unsigned long>(selectedRobots.at((i)).id) == static_cast<unsigned long>(robotId)) {
            robotWasAlreadySelected = true;
            this->selectedRobots.erase(selectedRobots.begin() + i);
        }
    }

    if (! robotWasAlreadySelected) {
        for (auto  &robot : rtt::ai::world::world->getUs()) {
            if (robot->id == robotId) {
                robotWasAlreadySelected = false;
                this->selectedRobots.push_back(*robot);
            }
        }
    }

}

bool Visualizer::robotIsSelected(Robot robotToCheck) {
    for (auto robot : selectedRobots) {
        if (robot.id == robotToCheck.id) return true;
    }
    return false;
}

bool Visualizer::robotIsSelected(int id) {
    for (auto robot : selectedRobots) {
        if (robot.id == id) return true;
    }
    return false;
}

void Visualizer::drawBallPlacementTarget(QPainter &painter) {
    Vector2 ballPlacementTarget = toScreenPosition(Output::getInterfaceMarkerPosition());
    painter.setBrush(Qt::transparent);
    painter.setPen(Qt::red);

    painter.drawLine(ballPlacementTarget.x - 5, ballPlacementTarget.y - 5, ballPlacementTarget.x + 5,
            ballPlacementTarget.y + 5);
    painter.drawLine(ballPlacementTarget.x + 5, ballPlacementTarget.y - 5, ballPlacementTarget.x - 5,
            ballPlacementTarget.y + 5);
}

void Visualizer::setShowBallPlacementMarker(bool showMarker) {
    Visualizer::showBallPlacementMarker = showMarker;
}

void Visualizer::setShowDebugValueInTerminal(bool showDebug) {
    Visualizer::showDebugValueInTerminal = showDebug;
    Output::setShowDebugValues(showDebug);
}

void Visualizer::setToggleFieldDirection(bool inversed) {
    Visualizer::fieldInversed = inversed;
}

void Visualizer::drawPlusses(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);

        // draw a plus
        painter.drawLine(0, pointOnScreen.y - height/2, 0, pointOnScreen.y + height/2);
        painter.drawLine(pointOnScreen.x + width/2, 0, pointOnScreen.x - width/2, 0);
    }
}

void Visualizer::drawArrows(QPainter &painter, std::vector<Vector2> points, double factor, double maxSize, bool closedArrow) {
    if (points.size() >= 2) {
        for (int i = 1; i < points.size(); i += 2) {
            Vector2 &arrowEnd = points.at(i-1);
            Vector2 &arrowStart = points.at(i);

            double arrowLength = (arrowEnd-arrowStart).length();
            Angle arrowAngle = (arrowEnd-arrowStart).toAngle();

            double arrowSizeFactor = factor == 4.0 ? 0.35 : std::min(1.0, factor);
            double maxArrowSize = maxSize == 4.0 ? 0.5 : std::min(1.0, maxSize);
            double arrowSize = arrowLength > maxArrowSize/arrowSizeFactor ? arrowSizeFactor : arrowSizeFactor*arrowLength;

            Vector2 startPoint = arrowEnd + (arrowStart-arrowEnd).stretchToLength(arrowSize);
            Vector2 pointyBitLeft = startPoint + (arrowAngle + M_PI_2).toVector2(arrowSize);
            Vector2 pointyBitRight = startPoint + (arrowAngle + M_PI_2).toVector2(-arrowSize);

            Vector2 arrowStartOnScreen = toScreenPosition(arrowStart);
            Vector2 arrowEndOnScreen = toScreenPosition(arrowEnd);
            Vector2 pointyBitLeftOnScreen = toScreenPosition(pointyBitLeft);
            Vector2 pointyBitRightOnScreen = toScreenPosition(pointyBitRight);
            Vector2 startPointOnScreen = toScreenPosition(startPoint);
            if (closedArrow) {
                painter.drawLine(arrowStartOnScreen.x, arrowStartOnScreen.y, startPointOnScreen.x, startPointOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitRightOnScreen.x, pointyBitRightOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
                painter.drawLine(pointyBitRightOnScreen.x, pointyBitRightOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
            }
            else {
                painter.drawLine(arrowStartOnScreen.x, arrowStartOnScreen.y, arrowEndOnScreen.x, arrowEndOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitRightOnScreen.x, pointyBitRightOnScreen.y);
                painter.drawLine(arrowEndOnScreen.x, arrowEndOnScreen.y, pointyBitLeftOnScreen.x, pointyBitLeftOnScreen.y);
            }

        }
    }
}

void Visualizer::drawCrosses(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawLine(pointOnScreen.x - width/2, pointOnScreen.y - height/2, pointOnScreen.x + width/2,
                pointOnScreen.y + height/2);
        painter.drawLine(pointOnScreen.x + width/2, pointOnScreen.y - height/2, pointOnScreen.x - width/2,
                pointOnScreen.y + height/2);
    }
}

void Visualizer::drawPoints(QPainter &painter, std::vector<Vector2> points, double width, double height) {
    for (auto const &point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawEllipse(pointOnScreen.x - width/2, pointOnScreen.y - height/2, width, height);
    }
}

void Visualizer::drawLines(QPainter &painter, std::vector<Vector2> points) {
    if (points.size() >= 2) {
        for (int i = 1; i < points.size(); i ++) {
            Vector2 pointOnScreen = toScreenPosition(points.at(i));
            Vector2 prevPointOnScreen = toScreenPosition(points.at(i - 1));
            painter.drawLine(pointOnScreen.x, pointOnScreen.y, prevPointOnScreen.x, prevPointOnScreen.y);
        }
    }
}

} // interface
} // ai
} // rtt

// QT performance improvement
#include "moc_widget.cpp"