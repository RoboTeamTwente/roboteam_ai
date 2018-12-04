//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "widget.h"
#include "drawer.h"

namespace c = rtt::ai::constants;

namespace rtt {
namespace ai {
namespace interface {

Visualizer::Visualizer(QWidget *parent) : QWidget(parent) { }

/// The update loop of the field widget. Invoked by widget->update();
void Visualizer::paintEvent(QPaintEvent* event) {
    QPainter painter(this);

    calculateFieldSizeFactor();
    if (rtt::ai::World::didReceiveFirstWorld) {
        drawBackground(painter);
        drawFieldLines(painter);
        drawBall(painter);
        drawRobots(painter);

        if (showPath) drawDataPoints(painter, Drawer::getGoToPosLuThPoints());
    } else {
        painter.drawText(24,24, "Waiting for incoming World State");
    }
}

/// Calculates the factor variable which is used for mapping field coordinates with screen coordinates.
void Visualizer::calculateFieldSizeFactor() {
    roboteam_msgs::GeometryFieldSize field = rtt::ai::Field::get_field();
    fieldmargin = static_cast<int>(c::WINDOW_FIELD_MARGIN + field.boundary_width);
    float widthFactor = this->size().width() / field.field_length - (2 * fieldmargin);
    float heightFactor = this->size().height() / field.field_width - (2 * fieldmargin);
    factor = std::min(widthFactor, heightFactor);
}

/// draws background of the field
void Visualizer::drawBackground(QPainter & painter) {
    painter.setBrush(c::FIELD_COLOR);
    painter.drawRect(0,0, this->size().width(), this->size().height());
}

// draws the field lines
void Visualizer::drawFieldLines(QPainter & painter) {
    painter.setPen(c::FIELD_LINE_COLOR);
    painter.setBrush(Qt::transparent);
    // draw lines
    for (auto &line : rtt::ai::Field::get_field().field_lines) {
        rtt::Vector2 start = toScreenPosition(line.begin);
        rtt::Vector2 end = toScreenPosition(line.end);
        painter.drawLine(start.x, start.y, end.x, end.y);
    }

    // draw the circle in the middle
    for (auto &arc : rtt::ai::Field::get_field().field_arcs) {
        rtt::Vector2 center = toScreenPosition(arc.center);
        QPointF qcenter(center.x, center.y);
        painter.drawEllipse(qcenter, 50, 50);
    }
}

// draw the ball on the screen
void Visualizer::drawBall(QPainter & painter) {
    rtt::Vector2 ballPosition = toScreenPosition(rtt::ai::World::get_world().ball.pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);
    painter.setBrush(c::BALL_COLOR); // fill
    painter.setPen(Qt::NoPen); // stroke
    painter.drawEllipse(qballPosition, c::BALL_DRAWING_SIZE, c::BALL_DRAWING_SIZE);
}

// draw the robots
void Visualizer::drawRobots(QPainter & painter) {

    // draw us
    for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().us) {
        drawRobot(painter, robot, true);
    }

    // draw them
    for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().them) {
        drawRobot(painter, robot, false);
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Visualizer::toScreenPosition(rtt::Vector2 fieldPos) {
    return {(fieldPos.x * factor) + static_cast<float>(this->size().width()/2 + fieldmargin),
            (fieldPos.y * factor * -1) + static_cast<float>(this->size().height()/2 + fieldmargin)};
}

// draw a single robot
void Visualizer::drawRobot(QPainter & painter, roboteam_msgs::WorldRobot robot, bool ourTeam) {
    Vector2 robotpos = toScreenPosition(robot.pos);
    QPointF qrobotPosition(robotpos.x, robotpos.y);
    QColor robotColor = ourTeam ? c::ROBOT_US_COLOR : c::ROBOT_THEM_COLOR;

    if (showAngles) {
        Vector2 angle = toScreenPosition({robot.pos.x + cos(robot.angle) / 3, robot.pos.y + sin(robot.angle) / 3});
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
        painter.setPen(c::TEXT_COLOR);
        painter.drawText(robotpos.x, ypos+=20, QString::fromStdString(getTacticNameForRobot(robot)));
    }

    if (showRoles && ourTeam) {
        painter.setPen(c::TEXT_COLOR);
        painter.drawText(robotpos.x, ypos+=20, QString::fromStdString(getRoleNameForRobot(robot)));
    }

    // draw the robots
    QColor color = (robot.id == selectedRobot.id && ourTeam) ? c::SELECTED_ROBOT_COLOR : robotColor;
    painter.setBrush(color);
    painter.setPen(Qt::transparent);
    painter.drawEllipse(qrobotPosition, c::ROBOT_DRAWING_SIZE, c::ROBOT_DRAWING_SIZE);

    // draw the id in it
    painter.setPen(Qt::black);
    painter.drawText(robotpos.x-3, robotpos.y+5, QString::fromStdString(std::to_string(robot.id)));
}


// Handle mousePressEvents
void Visualizer::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        Vector2 pos;
        pos.x = event->pos().x();
        pos.y = event->pos().y();

        for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().us) {
            if (pos.dist(toScreenPosition(robot.pos)) < 10) {
                this->selectedRobot = robot;
            }
        }
    }
}

void Visualizer::drawTacticColorForRobot(QPainter & painter, roboteam_msgs::WorldRobot robot) {
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

    if (!tacticExists) {
        QColor newColor = c::TACTIC_COLORS[tacticCount];
        tacticCount = (tacticCount + 1) % sizeof(c::TACTIC_COLORS);
        tacticColors.push_back({tacticName, newColor});
        c = newColor;
    }

    painter.setPen(Qt::transparent);
    painter.setBrush(c);
    painter.drawEllipse(qrobotPosition, c::TACTIC_COLOR_DRAWING_SIZE, c::TACTIC_COLOR_DRAWING_SIZE);
}

void Visualizer::drawDataPoints(QPainter & painter, std::vector<Vector2> points, int pointSize, QColor color) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);

    for (Vector2 point : points) {
        Vector2 pointOnScreen = toScreenPosition(point);
        painter.drawEllipse(pointOnScreen.x, pointOnScreen.y, pointSize, pointSize);
    }
}

std::string Visualizer::getTacticNameForRobot(roboteam_msgs::WorldRobot robot) {
    for (auto &robotowner : robotDealer::RobotDealer::getClaimedRobots()) {
        std::set<std::pair<int, std::string>> robots = robotowner.second;
        for (auto &ownedRobot : robots) {
            if (ownedRobot.first == robot.id) {
                return robotowner.first;
            }
        }
    }
    return "";
}

std::string Visualizer::getRoleNameForRobot(roboteam_msgs::WorldRobot robot) {
    for (auto &robotowner : robotDealer::RobotDealer::getClaimedRobots()) {
        std::set<std::pair<int, std::string>> robots = robotowner.second;
        for (auto &ownedRobot : robots) {
            if (ownedRobot.first == robot.id) {
                return ownedRobot.second;
            }
        }
    }
    return "";
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

const roboteam_msgs::WorldRobot &Visualizer::getSelectedRobot() const {
    return selectedRobot;
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


} // interface
} // ai
} // rtt
