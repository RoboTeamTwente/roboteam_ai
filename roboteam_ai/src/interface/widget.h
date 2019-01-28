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
#include <QMouseEvent>
#include <gtest/gtest_prod.h>

namespace rtt {
namespace ai {
namespace interface {

class Visualizer : public QWidget {
    Q_OBJECT
    FRIEND_TEST(VisualizerTest, it_shows_proper_data);
    public:
        explicit Visualizer(QWidget* parent = nullptr);
        const  std::vector<roboteam_msgs::WorldRobot> &getSelectedRobots() const;
        bool robotIsSelected(roboteam_msgs::WorldRobot robot);

    public slots:
        void setShowRoles(bool showRoles);
        void setShowTactics(bool showTactics);
        void setShowTacticColors(bool showTacticColors);
        void setShowAngles(bool showAngles);
        void setShowVelocities(bool showVelocities);
        void setShowPath(bool showPath);
        void setShowPathAll(bool showPaths);
        void setShowBallPlacementMarker(bool showMarker);

    void toggleSelectedRobot(int robotId);
    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;

    private:
        float factor;
        int fieldmargin = constants::WINDOW_FIELD_MARGIN;
        void drawBackground(QPainter &painter);
        void drawFieldLines(QPainter &painter);
        void drawRobots(QPainter &painter);
        void drawRobot(QPainter &painter, roboteam_msgs::WorldRobot robot, bool ourTeam);
        void drawBall(QPainter &painter);
        void drawBallPlacementTarget(QPainter &painter);
        void drawTacticColorForRobot(QPainter &painter, roboteam_msgs::WorldRobot robot);
        void drawDataPoints(QPainter &painter, std::vector<Vector2> points, int pointSize = 3,
                QColor color = Qt::green);
        void drawDataPoints(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points, int pointSize = 3);
        // utitlity functions
        std::string getTacticNameForRobot(roboteam_msgs::WorldRobot robot);
        std::string getRoleNameForRobot(roboteam_msgs::WorldRobot robot);
        rtt::Vector2 toScreenPosition(rtt::Vector2 fieldPos);
        rtt::Vector2 toFieldPosition(rtt::Vector2 screenPos);

        void calculateFieldSizeFactor();
        void drawIntercept(QPainter &painter, std::vector<std::pair<Vector2, QColor>> points);
        // interface variables
        std::vector<std::pair<std::string,
                              QColor>> tacticColors; // map colors to tactic to visualize which robots work together
        int tacticCount = 0; // increases when a new tactic is used

        std::vector<roboteam_msgs::WorldRobot> selectedRobots;

        // toggles
        bool showRoles = constants::STD_SHOW_ROLES;
        bool showTactics = constants::STD_SHOW_TACTICS;
        bool showTacticColors = constants::STD_SHOW_TACTICS_COLORS;
        bool showAngles = constants::STD_SHOW_ANGLES;
        bool showVelocities = constants::STD_SHOW_VELOCITIES;
        bool showPath = constants::STD_SHOW_PATHS_CURRENT;
        bool showAllPaths = constants::STD_SHOW_PATHS_ALL;
        bool showBallPlacementMarker = constants::STD_SHOW_BALL_PLACEMENT_MARKER;

};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_WIDGET_H
