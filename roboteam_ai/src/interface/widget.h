//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_WIDGET_H
#define ROBOTEAM_AI_WIDGET_H

#include <QWidget>
#include <QPainter>

class Widget : public QWidget
{
    Q_OBJECT
    public:
        explicit Widget(QWidget *parent = nullptr);

    protected:
        void paintEvent(QPaintEvent *event) {
            QPainter painter(this);
            painter.setPen(QPen(Qt::white, 12, Qt::DashDotLine, Qt::RoundCap));
            painter.drawLine(0, 0, 200, 200);
        }

    signals:

    public slots:
};

#endif //ROBOTEAM_AI_WIDGET_H
