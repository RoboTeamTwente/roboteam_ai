//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_WIDGET_H
#define ROBOTEAM_AI_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <memory>
#include "../utilities/Constants.h"
#include "../utilities/Field.h"
#include "../utilities/World.h"

class Widget : public QWidget {
    Q_OBJECT
    public:
        explicit Widget(QWidget* parent = nullptr);

    protected:
        void paintEvent(QPaintEvent* event);
    signals:

    public slots:

    private:

        rtt::Vector2 factor;
        int fieldmargin;
        void drawFieldLines();
        void drawFieldArcs();
        void drawRobots();
        void drawBall();
        rtt::Vector2 toScreenPosition(rtt::Vector2 fieldPos);
};

#endif //ROBOTEAM_AI_WIDGET_H
