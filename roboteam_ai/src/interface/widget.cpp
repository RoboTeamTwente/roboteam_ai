//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "widget.h"

namespace c = rtt::ai::constants;

namespace rtt {
namespace ai {
namespace interface {

Widget::Widget(QWidget *parent) : QWidget(parent) { }

/// The update loop of the field widget. Invoked by widget->update();
void Widget::paintEvent(QPaintEvent* event) {
    calculateFieldSizeFactor();
    if (rtt::ai::World::didReceiveFirstWorld) {
        drawBackground();
        drawFieldLines();
        drawBall();
        drawRobots();
    } else {
        QPainter painter(this);
        painter.drawText(24,24, "Waiting for incoming World State");
    }
}

/// Calculates the factor variable which is used for mapping field coordinates with screen coordinates.
void Widget::calculateFieldSizeFactor() {
    roboteam_msgs::GeometryFieldSize field = rtt::ai::Field::get_field();
    fieldmargin = static_cast<int>(c::WINDOW_FIELD_MARGIN + field.boundary_width);
    float widthFactor = this->size().width() / field.field_length - (2 * fieldmargin);
    float heightFactor = this->size().height() / field.field_width - (2 * fieldmargin);
    factor = std::min(widthFactor, heightFactor);
}

/// draws background of the field
void Widget::drawBackground() {
    QPainter painter(this);
    painter.setBrush(Qt::darkGreen);
    painter.drawRect(0,0, this->size().width(), this->size().height());
}

// draws the field lines
void Widget::drawFieldLines() {
    QPainter painter(this);
    painter.setPen(Qt::white);

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
void Widget::drawBall() {
    QPainter painter(this);
    rtt::Vector2 ballPosition = toScreenPosition(rtt::ai::World::get_world().ball.pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);
    painter.setBrush(Qt::red); // fill
    painter.setPen(Qt::NoPen); // stroke
    painter.drawEllipse(qballPosition, 5, 5);
}

// draw the robots
void Widget::drawRobots() {

    // draw us
    for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().us) {
        drawRobot(robot, true);
    }

    // draw them
    for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().them) {
        drawRobot(robot, false);
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Widget::toScreenPosition(rtt::Vector2 fieldPos) {
    return {(fieldPos.x * factor) + static_cast<float>(this->size().width()/2 + fieldmargin),
            (fieldPos.y * factor * -1) + static_cast<float>(this->size().height()/2 + fieldmargin)};
}

// draw a single robot
void Widget::drawRobot(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    QPainter painter(this);
    rtt::Vector2 robotposition = toScreenPosition(robot.pos);
    QPointF qrobotPosition(robotposition.x, robotposition.y);
    Vector2 robotpos = toScreenPosition(robot.pos);
    QColor robotColor = ourTeam ? Qt::yellow : Qt::blue;

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


    if (ourTeam) {
        std::map<std::string, std::set<std::pair<int, std::string>>> list = robotDealer::RobotDealer::getClaimedRobots();

        std::string roleName;
        std::string tacticName;
        for (auto &robotowner : list) {
            std::set<std::pair<int, std::string>> robots = robotowner.second;
            for (auto &ownedRobot : robots) {
                if (ownedRobot.first == robot.id) {
                    tacticName = robotowner.first;
                    roleName = ownedRobot.second;
                }
            }
        }

        if (showTacticColors) {
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

            QPen pen;
            pen.setWidth(3);
            pen.setBrush(c);
            painter.setPen(pen);
        } else {
            painter.setPen(Qt::transparent);
        };

        if (robot.id == selectedRobot.id) {
            painter.setBrush(Qt::magenta);
        } else {
            painter.setBrush(Qt::yellow);
        }
        painter.drawEllipse(qrobotPosition, 10, 10);

        // draw text
        painter.setPen(Qt::black);
        rtt::Vector2 pos = toScreenPosition(robot.pos);
        int ypos = pos.y;
        painter.drawText(pos.x-3, ypos+5, QString::fromStdString(std::to_string(robot.id)));
        if (this->showTactics) painter.drawText(pos.x, ypos+=20, QString::fromStdString(tacticName));
        if (this->showRoles) painter.drawText(pos.x, ypos+=20, QString::fromStdString(roleName));
    }
}


// Handle mousePressEvents
void Widget::mousePressEvent(QMouseEvent *event) {
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

void Widget::setShowRoles(bool showRoles) {
    this->showRoles = showRoles;
}

void Widget::setShowTactics(bool showTactics) {
    Widget::showTactics = showTactics;
}

void Widget::setShowTacticColors(bool showTacticColors) {
    Widget::showTacticColors = showTacticColors;
}

const roboteam_msgs::WorldRobot &Widget::getSelectedRobot() const {
    return selectedRobot;
}

void Widget::setShowAngles(bool showAngles) {
    Widget::showAngles = showAngles;
}

void Widget::setShowVelocities(bool showVelocities) {
    Widget::showVelocities = showVelocities;
}


} // interface
} // ai
} // rtt
