//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <ros/node_handle.h>
#include "widget.h"
#include "drawer.h"
#include "InterfaceValues.h"
#include "../analysis/GameAnalyzer.h"

namespace rtt {
namespace ai {
namespace interface {

Visualizer::Visualizer(QWidget* parent) : QWidget(parent) { }

/// The update loop of the field widget. Invoked by widget->update();
void Visualizer::paintEvent(QPaintEvent* event) {
    QPainter painter(this);

    calculateFieldSizeFactor();
    if (rtt::ai::world::world->weHaveRobots()) {
        drawBackground(painter);
        drawFieldLines(painter);
        if (showAvailablePasses) drawPasses(painter);
        drawBall(painter);
        drawRobots(painter);
        drawLines(painter,Drawer::getTestLines());
        drawPoints(painter,Drawer::getTestPoints());
        drawDrawPoints(painter, Drawer::getDrawPoints());
        drawDrawLines(painter, Drawer::getDrawLines());

        Drawer::clearDrawPoints();
        Drawer::clearDrawLines();

        if (showBallPlacementMarker) drawBallPlacementTarget(painter);
        if (showPath) {
            for (auto robot : selectedRobots) {
                drawDataPoints(painter, Drawer::getNumTreePoints(robot.id));
                drawDataPoints(painter, Drawer::getKeeperPoints(robot.id),Constants::KEEPER_HELP_DRAW_SIZE());
                drawIntercept(painter, Drawer::getInterceptPoints(robot.id));
            }
        }

    }
    else {
        painter.drawText(24, 24, "Waiting for incoming World State");
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
    painter.drawRect(-10, -10, this->size().width() + 10, this->size().height() +10);
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
    for (auto &arc : rtt::ai::world::field->get_field().field_arcs) {
        rtt::Vector2 center = toScreenPosition(arc.center);
        QPointF qcenter(center.x, center.y);
        painter.drawEllipse(qcenter, 50, 50);
    }
}

// draw the ball on the screen
void Visualizer::drawBall(QPainter &painter) {
    rtt::Vector2 ballPosition = toScreenPosition(rtt::ai::world::world->getWorld().ball.pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);
    if (!rtt::ai::world::world->getWorld().ball.visible){
        painter.setBrush(Qt::red); // fill
    }
    else{
        painter.setBrush(Constants::BALL_COLOR()); // fill
    }
    painter.setPen(Qt::NoPen); // stroke
    painter.drawEllipse(qballPosition, Constants::BALL_DRAWING_SIZE(), Constants::BALL_DRAWING_SIZE());
}

// draw the robots
void Visualizer::drawRobots(QPainter &painter) {

    // draw us
    for (auto &robot : rtt::ai::world::world->getWorld().us) {
        drawRobot(painter, robot, true);
    }

    // draw them
    for (auto &robot : rtt::ai::world::world->getWorld().them) {
        drawRobot(painter, robot, false);
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toScreenPosition(rtt::Vector2 fieldPos) {
    int inv = fieldInversed ? -1 : 1;
    return {(fieldPos.x*factor * inv) + static_cast<float>(this->size().width()/2 + fieldmargin) ,
            (fieldPos.y*factor*- 1) + static_cast<float>(this->size().height()/2 + fieldmargin)};



}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toFieldPosition(rtt::Vector2 screenPos) {
        int inv = fieldInversed ? -1 : 1;

    auto x = ((screenPos.x * inv) - fieldmargin - static_cast<float>(this->size().width()/2)) / factor;
    auto y = ((screenPos.y - fieldmargin - static_cast<float>(this->size().height()/2)) / factor) * -1;

    return {x,y};
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
    } else {
        // the enemy robot should have the inverse of our_color
        robotColor = weAreYellow ? Constants::ROBOT_COLOR_BLUE() : Constants::ROBOT_COLOR_YELLOW();
    }

    if (ourTeam && robot.id == robotDealer::RobotDealer::getKeeperID()) {
        robotColor = QColor(255, 255, 255);

    }

    if (showAllPaths) {
        std::vector<rtt::Vector2> gtpltPoints;
        for (auto pair : Drawer::getNumTreePoints(robot.id)) {
            gtpltPoints.push_back(pair.first);
        }
        drawDataPoints(painter, gtpltPoints, 2, Qt::gray);
    }

    if (showAngles) {
        Vector2 angle = toScreenPosition({robot.pos.x + cos(robot.angle)/3, robot.pos.y + sin(robot.angle)/3});
        QPen pen;
        pen.setWidth(4);
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

    // draw the robots
    QColor color = (robotIsSelected(robot) && ourTeam) ? Constants::SELECTED_ROBOT_COLOR() : robotColor;
    painter.setBrush(color);
    painter.setPen(Qt::transparent);
    painter.drawEllipse(qrobotPosition, Constants::ROBOT_DRAWING_SIZE(), Constants::ROBOT_DRAWING_SIZE());

    // draw the id in it
    painter.setPen(Qt::black);
    painter.drawText(robotpos.x - 3, robotpos.y + 5, QString::fromStdString(std::to_string(robot.id)));
}

// Handle mousePressEvents
void Visualizer::mousePressEvent(QMouseEvent* event) {
    Vector2 pos;
    pos.x = event->pos().x();
    pos.y = event->pos().y();

    if (event->button() == Qt::LeftButton) {
        for (auto &robot : rtt::ai::world::world->getWorld().us) {
            if (pos.dist(toScreenPosition(robot.pos)) < 10) {
                this->toggleSelectedRobot(robot.id);
            }
        }
    } else if (event->button() == Qt::RightButton) {
        InterfaceValues::setBallPlacementTarget(toFieldPosition(pos));

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
        tacticCount = (tacticCount + 1)% Constants::TACTIC_COLORS().size();
        tacticColors.push_back({tacticName, newColor});
        c = newColor;
    }

    painter.setPen(Qt::transparent);
    painter.setBrush(c);
    painter.drawEllipse(qrobotPosition, Constants::TACTIC_COLOR_DRAWING_SIZE(), Constants::TACTIC_COLOR_DRAWING_SIZE());
}

void Visualizer::drawDataPoints(QPainter &painter, std::vector<Vector2> points, int pointSize, QColor color) {
    if (! points.empty()) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);

        for (Vector2 point : points) {
            Vector2 pointOnScreen = toScreenPosition(point);
            painter.drawEllipse(pointOnScreen.x, pointOnScreen.y, pointSize, pointSize);
        }
    }
}

void Visualizer::drawDataPoints(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, int pointSize) {
    if (! points.empty()) {
        painter.setPen(Qt::NoPen);

        for (auto point : points) {
            painter.setBrush(point.second);
            Vector2 pointOnScreen = toScreenPosition(point.first);
            painter.drawEllipse(pointOnScreen.x, pointOnScreen.y, pointSize, pointSize);
        }
    }
}

void Visualizer::drawCrosses(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, double size) {
    if (!points.empty()) {
        for (auto point : points) {
            painter.setPen(point.second);
            Vector2 pointOnScreen = toScreenPosition(point.first);
            painter.drawLine(pointOnScreen.x - size, pointOnScreen.y - size, pointOnScreen.x + size,
                             pointOnScreen.y + size);
            painter.drawLine(pointOnScreen.x + size, pointOnScreen.y - size, pointOnScreen.x - size,
                             pointOnScreen.y + size);
        }
    }
}

void Visualizer::drawDrawPoints(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, int pointSize) {
    if (! points.empty()) {
        painter.setPen(Qt::NoPen);

        for (auto point : points) {
            painter.setBrush(point.second);
            Vector2 pointOnScreen = toScreenPosition(point.first);
            painter.drawEllipse(pointOnScreen.x, pointOnScreen.y, pointSize, pointSize);
        }
    }
}

void Visualizer::drawDrawLines(QPainter &painter, std::vector<std::tuple<Vector2, Vector2, QColor>> lines) {
    if (!lines.empty()) {
        for (auto &line : lines) {
            painter.setPen(std::get<2>(line));
            Vector2 v1 = toScreenPosition(std::get<0>(line));
            Vector2 v2 = toScreenPosition(std::get<1>(line));
            painter.drawLine(v1.x, v1.y, v2.x, v2.y);
        }
    }
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

void Visualizer::setShowPath(bool showPath) {
    Visualizer::showPath = showPath;
}

void Visualizer::setShowPathAll(bool showPaths) {
    Visualizer::showAllPaths = showPaths;
}

void Visualizer::toggleSelectedRobot(int robotId) {
    bool robotWasAlreadySelected = false;

    for (int i = 0; i < static_cast<int>(selectedRobots.size()); i++) {
        if (static_cast<unsigned long>(selectedRobots.at((i)).id) == static_cast<unsigned long>(robotId)) {
            robotWasAlreadySelected = true;
            this->selectedRobots.erase(selectedRobots.begin() + i);
        }
    }

    if (!robotWasAlreadySelected) {
        for (Robot robot : rtt::ai::world::world->getWorld().us) {
            if (robot.id == robotId) {
                robotWasAlreadySelected = false;
                this->selectedRobots.push_back(robot);
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

void Visualizer::drawIntercept(QPainter &painter, std::vector<std::pair<rtt::Vector2, QColor>> points) {
    if (! points.empty()) {
        for (int j = 0; j < Constants::INTERCEPT_DRAW_VECTOR_SIZE(); ++ j) {
            //first point needs to be drawn as a standalone (is not a line)
            if (j < 1) {
                Vector2 PointZ = toScreenPosition(points[j].first);
                painter.setPen(points[j].second);
                painter.drawEllipse(PointZ.x, PointZ.y, Constants::KEEPER_HELP_DRAW_SIZE(),
                        Constants::KEEPER_HELP_DRAW_SIZE());
            }
            else {
                painter.setPen(points[j].second);
                Vector2 pointA = toScreenPosition(points[j].first);
                Vector2 pointB = toScreenPosition(points[j + 1].first);
                painter.drawLine(pointA.x, pointA.y, pointB.x, pointB.y);
                ++ j;

            }
        }
    }
}
void Visualizer::drawLines(QPainter &painter, std::vector<std::pair<std::pair<rtt::Vector2,rtt::Vector2>,QColor>> lines){
    for (auto line : lines){
        Vector2 start=toScreenPosition(line.first.first);
        Vector2 end=toScreenPosition(line.first.second);
        painter.setPen(line.second);
        painter.drawLine(start.x,start.y,end.x,end.y);
    }
}
void Visualizer::drawPoints(QPainter &painter, std::vector<std::pair<Vector2,QColor>> points){
    for (auto point : points){
        Vector2 screenPoint=toScreenPosition(point.first);
        painter.setPen(point.second);
        painter.drawEllipse(screenPoint.x-2,screenPoint.y-2,4,4);
    }
}

void Visualizer::drawBallPlacementTarget(QPainter& painter) {
    Vector2 ballPlacementTarget = toScreenPosition(InterfaceValues::getBallPlacementTarget());
    painter.setBrush(Qt::transparent);
    painter.setPen(Qt::red);
    painter.drawLine(ballPlacementTarget.x - 5, ballPlacementTarget.y - 5, ballPlacementTarget.x + 5, ballPlacementTarget.y + 5);
    painter.drawLine(ballPlacementTarget.x + 5, ballPlacementTarget.y - 5, ballPlacementTarget.x - 5, ballPlacementTarget.y + 5);
}

void Visualizer::setShowBallPlacementMarker(bool showMarker) {
    Visualizer::showBallPlacementMarker = showMarker;
}

void Visualizer::setShowDebugValueInTerminal(bool showDebug) {
    Visualizer::showDebugValueInTerminal = showDebug;
    InterfaceValues::setShowDebugValues(showDebug);
}

void Visualizer::setShowAvailablePasses(bool showAvailablePasses) {
    Visualizer::showAvailablePasses = showAvailablePasses;
}

void Visualizer::drawPasses(QPainter& painter) {
    auto report = rtt::ai::analysis::GameAnalyzer::getInstance().getMostRecentReport();
if (report) {
    std::vector<std::pair<Vector2, Vector2>> lines;
    for (auto &robot : report->ourRobotsSortedOnDanger) {
        if (robotIsSelected(robot.first)) {
            Vector2 robotLocation = toScreenPosition(robot.first.pos);
            for (auto robotToPassToId : robot.second.robotsToPassTo) {
                auto passRobot = world::world->getRobotForId(robotToPassToId.first, true);
                Vector2 passRobotLocation = toScreenPosition(passRobot->pos);
                double distance = robotToPassToId.second;
                painter.setBrush(Qt::transparent);
                int opacity = static_cast<int>((robotLocation.dist(passRobotLocation) / width()) * 255);
                painter.setPen({255, 255, 0, 255 - opacity});
                painter.drawLine(robotLocation.x, robotLocation.y, passRobotLocation.x, passRobotLocation.y);
            }
        }
    };
}
}

    void Visualizer::setToggleFieldDirection(bool inversed) {
        Visualizer::fieldInversed = inversed;
    }

} // interface
} // ai
} // rtt
