//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_WIDGET_H
#define ROBOTEAM_AI_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <memory>
#include "../utilities/Constants.h"
#include "roboteam_ai/src/world/Field.h"
#include "../world/World.h"
#include <QMouseEvent>
#include <gtest/gtest_prod.h>
#include "roboteam_ai/src/world/WorldData.h"

namespace rtt {
namespace ai {
namespace interface {

class Visualizer : public QWidget {
    Q_OBJECT
    FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);
    public:
        using Robot = rtt::ai::world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        explicit Visualizer(QWidget* parent = nullptr);
        const  std::vector<Robot> &getSelectedRobots() const;
        bool robotIsSelected(Robot robotToCheck);

    public slots:
        void setShowRoles(bool showRoles);
        void setShowTactics(bool showTactics);
        void setShowTacticColors(bool showTacticColors);
        void setShowAngles(bool showAngles);
        void setShowVelocities(bool showVelocities);
        void setShowPath(bool showPath);
        void setShowPathAll(bool showPaths);
        void setShowBallPlacementMarker(bool showMarker);
        void setShowDebugValueInTerminal(bool showDebug);
        void toggleSelectedRobot(int robotId);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;

    private:
        float factor;
        int fieldmargin = Constants::WINDOW_FIELD_MARGIN();
        void drawBackground(QPainter &painter);
        void drawFieldLines(QPainter &painter);
        void drawRobots(QPainter &painter);
        void drawRobot(QPainter &painter, Robot, bool ourTeam);
        void drawBall(QPainter &painter);
        void drawBallPlacementTarget(QPainter &painter);
        void drawTacticColorForRobot(QPainter &painter, Robot robot);
        void drawDataPoints(QPainter &painter, std::vector<Vector2> points, int pointSize = 3,
                QColor color = Qt::green);
        void drawDataPoints(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, int pointSize = 3);

        void drawDrawPoints(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, int pointSize = 3);
        void drawDrawLines(QPainter &painter, std::vector<std::tuple<Vector2, Vector2, QColor>> lines);

        // utitlity functions
        std::string getTacticNameForRobot(Robot robot);
        std::string getRoleNameForRobot(Robot robot);
        rtt::Vector2 toScreenPosition(rtt::Vector2 fieldPos);
        rtt::Vector2 toFieldPosition(rtt::Vector2 screenPos);

        void calculateFieldSizeFactor();
        void drawIntercept(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points);
        // interface variables
        std::vector<std::pair<std::string,
                              QColor>> tacticColors; // map colors to tactic to visualize which robots work together
        int tacticCount = 0; // increases when a new tactic is used

        std::vector<Robot> selectedRobots;

        // toggles
        bool showRoles = Constants::STD_SHOW_ROLES();
        bool showTactics = Constants::STD_SHOW_TACTICS();
        bool showTacticColors = Constants::STD_SHOW_TACTICS_COLORS();
        bool showAngles = Constants::STD_SHOW_ANGLES();
        bool showVelocities = Constants::STD_SHOW_VELOCITIES();
        bool showPath = Constants::STD_SHOW_PATHS_CURRENT();
        bool showAllPaths = Constants::STD_SHOW_PATHS_ALL();
        bool showBallPlacementMarker = Constants::STD_SHOW_BALL_PLACEMENT_MARKER();
        bool showDebugValueInTerminal = Constants::STD_SHOW_DEBUG_VALUES();
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_WIDGET_H
