//
// Created by mrlukasbos on 27-11-18.
//

#include "widget.h"

namespace c = rtt::ai::constants;

Widget::Widget(QWidget *parent) : QWidget(parent) { }

void Widget::paintEvent(QPaintEvent* event) {
    roboteam_msgs::GeometryFieldSize field = rtt::ai::Field::get_field();
    fieldmargin = static_cast<int>(c::WINDOW_FIELD_MARGIN + field.boundary_width);
    factor.x = c::WINDOW_SIZE_X / field.field_length - (2 * fieldmargin);
    factor.y = c::WINDOW_SIZE_Y / field.field_width - (2 * fieldmargin);

    drawFieldLines();
    drawFieldArcs();
    drawBall();
    drawRobots();

}

void Widget::drawFieldLines() {
    QPainter painter(this);
    for (auto line : rtt::ai::Field::get_field().field_lines) {
        rtt::Vector2 start = toScreenPosition(line.begin);
        rtt::Vector2 end = toScreenPosition(line.end);
        painter.drawLine(start.x, start.y, end.x, end.y);
    }
}

void Widget::drawFieldArcs() {
    QPainter painter(this);
    for (auto arc : rtt::ai::Field::get_field().field_arcs) {
        rtt::Vector2 center = toScreenPosition(arc.center);
        QPointF qcenter(center.x, center.y);
        painter.drawEllipse(qcenter, 50, 50);
    }
}

void Widget::drawBall() {
    QPainter painter(this);
    rtt::Vector2 ballPosition = toScreenPosition(rtt::ai::World::get_world().ball.pos);
    QPointF qballPosition(ballPosition.x, ballPosition.y);
    painter.setBrush(Qt::red);
    painter.drawEllipse(qballPosition, 5, 5);
}

void Widget::drawRobots() {
    QPainter painter(this);

    // draw us
    for (roboteam_msgs::WorldRobot robot : rtt::ai::World::get_world().us) {
        rtt::Vector2 robotposition = toScreenPosition(robot.pos);
        QPointF qrobotPosition(robotposition.x, robotposition.y);
        painter.setBrush(Qt::black);
        painter.drawEllipse(qrobotPosition, 7, 7);
    }
}

// convert field coordinates to screen coordinates
rtt::Vector2 Widget::toScreenPosition(rtt::Vector2 fieldPos) {
    return {(fieldPos.x * factor.x) + static_cast<float>(c::WINDOW_SIZE_X/2 + fieldmargin),
            (fieldPos.y * factor.y * -1) + static_cast<float>(c::WINDOW_SIZE_Y/2 + fieldmargin)};
}