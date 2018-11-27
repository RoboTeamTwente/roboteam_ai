//
// Created by mrlukasbos on 27-11-18.
//

#include "visualizer.h"

Widget::Widget(QWidget *parent) : QWidget(parent) {

}
void Widget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setPen(QPen(Qt::white, 12, Qt::DashDotLine, Qt::RoundCap));
    painter.drawLine(0, 0, 200, 200);
}
