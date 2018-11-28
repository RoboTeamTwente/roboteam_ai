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

namespace rtt {
namespace ai {
namespace interface {

class Widget : public QWidget {
    Q_OBJECT
    public:
        explicit Widget(QWidget* parent = nullptr);

        bool showRoles = constants::STD_SHOW_ROLES;
        bool showTactics = constants::STD_SHOW_TACTICS;
        bool showTacticColors = constants::STD_SHOW_TACTICS_COLORS;

        void setShowTactics(bool showTactics);
        void setShowTacticColors(bool showTacticColors);

    protected:
        void paintEvent(QPaintEvent* event);
    signals:

    public slots:
        void setShowRoles(bool showRoles);

    private:
        float factor;
        int fieldmargin;
        void drawBackground();
        void drawFieldLines();
        void drawFieldArcs();
        void drawRobots();
        void drawRobot(roboteam_msgs::WorldRobot robot, bool ourTeam);
        void drawBall();
        rtt::Vector2 toScreenPosition(rtt::Vector2 fieldPos);

        // map colors to tactic to visualize which robots work together
        std::vector<std::pair<std::string, SDL_Color>> tacticColors;
        int tacticCount = 0; // increases when a new tactic is used
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_WIDGET_H
