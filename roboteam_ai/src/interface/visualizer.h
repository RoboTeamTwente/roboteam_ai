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
        void paintEvent(QPaintEvent *event);
    signals:

    public slots:
};

#endif //ROBOTEAM_AI_WIDGET_H
