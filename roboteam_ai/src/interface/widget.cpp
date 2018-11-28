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

void Widget::paintEvent(QPaintEvent* event) {
    roboteam_msgs::GeometryFieldSize field = rtt::ai::Field::get_field();
    fieldmargin = static_cast<int>(c::WINDOW_FIELD_MARGIN + field.boundary_width);
    float widthFactor = this->size().width() / field.field_length - (2 * fieldmargin);
    float heightFactor = this->size().height() / field.field_width - (2 * fieldmargin);

    factor = std::min(widthFactor, heightFactor);

    drawBackground();
    if (rtt::ai::World::didReceiveFirstWorld) {
        drawFieldLines();
        drawFieldArcs();
        drawBall();
        drawRobots();
    } else {
        QPainter painter(this);
        painter.drawText(24,24, "Waiting for incoming World State");
    }
}

void Widget::drawBackground() {
    QPainter painter(this);
    painter.setBrush(Qt::darkGreen);
    painter.drawRect(0,0, this->size().width(), this->size().height());
}

void Widget::drawFieldLines() {
    QPainter painter(this);
    for (auto line : rtt::ai::Field::get_field().field_lines) {
        rtt::Vector2 start = toScreenPosition(line.begin);
        rtt::Vector2 end = toScreenPosition(line.end);
        painter.setPen(Qt::white);
        painter.drawLine(start.x, start.y, end.x, end.y);
    }
}

void Widget::drawFieldArcs() {
    QPainter painter(this);
    for (auto arc : rtt::ai::Field::get_field().field_arcs) {
        rtt::Vector2 center = toScreenPosition(arc.center);
        QPointF qcenter(center.x, center.y);
        painter.setPen(Qt::white);
        painter.drawEllipse(qcenter, 50, 50);
    }
}

void Widget::drawBall() {
    QPainter painter(this);
    rtt::Vector2 ballPosition = toScreenPosition(rtt::ai::World::get_world().ball.pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);
    painter.setBrush(Qt::red); // fill
    painter.setPen(Qt::NoPen); // stroke
    painter.drawEllipse(qballPosition, 5, 5);
}

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

void Widget::drawRobot(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    QPainter painter(this);

    std::map<std::string, std::set<std::pair<int, std::string>>> list = robotDealer::RobotDealer::getClaimedRobots();


    rtt::Vector2 robotposition = toScreenPosition(robot.pos);
    QPointF qrobotPosition(robotposition.x, robotposition.y);

    if (ourTeam) {
        painter.setBrush(Qt::yellow);
    } else {
        painter.setBrush(Qt::blue);
    }

    if (ourTeam) {
        std::string roleName;
        for (auto &robotowner : list) {
            std::string tactic = robotowner.first;
            std::set<std::pair<int, std::string>> robots = robotowner.second;
            for (auto &ownedRobot : robots) {
                if (ownedRobot.first == robot.id) {
                    roleName = ownedRobot.second;

                    bool tacticExists = false;
                    SDL_Color c;
                    for (auto tac : tacticColors) {
                        if (tac.first == tactic) {
                            c = tac.second;
                            tacticExists = true;
                            break;
                        }
                    }

                    if (!tacticExists) {
                        SDL_Color newColor = c::TACTIC_COLORS[tacticCount];
                        tacticCount = (tacticCount + 1) % sizeof(c::TACTIC_COLORS);
                        tacticColors.push_back({tactic, newColor});
                        c = newColor;
                    }

                   // drawRect(robot.pos, c::ROBOT_DRAWING_SIZE+4, c::ROBOT_DRAWING_SIZE+4, c);
                    rtt::Vector2 pos = toScreenPosition(robot.pos);

                    if (this->showTactics) {
                        painter.drawText(pos.x, pos.y - 20, QString::fromStdString(tactic));
                    }
                    if (this->showRoles) {
                        painter.drawText(pos.x, pos.y - 40, QString::fromStdString(roleName));
                    }
                }
            }
        }
    }

    // no stroke
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(qrobotPosition, 7, 7);
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

} // interface
} // ai
} // rtt
