//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_WIDGET_H
#define ROBOTEAM_AI_WIDGET_H

#include <gtest/gtest_prod.h>
#include <QMouseEvent>
#include <QPainter>
#include <QWidget>
#include <memory>

#include <interface/api/Toggles.h>
#include <roboteam_utils/Line.h>
#include <roboteam_utils/Vector2.h>
#include <world/Field.h>
#include <include/roboteam_ai/world_new/World.hpp>

namespace rtt::ai::interface {

class Visualizer : public QWidget {
    Q_OBJECT
    FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);

   public:
    explicit Visualizer(const rtt::world_new::World &worldManager, QWidget *parent = nullptr);
    const std::unordered_map<int, rtt::world_new::view::RobotView> &getSelectedRobots() const;
    bool robotIsSelected(rtt::world_new::view::RobotView robot);
    bool robotIsSelected(int id);
    void setPlayForRobot(std::string const& view, uint8_t i);

   public slots:
    void setShowRoles(bool showRoles);
    void setShowTactics(bool showTactics);
    void setShowTacticColors(bool showTacticColors);
    void setShowAngles(bool showAngles);
    void setShowVelocities(bool showVelocities);
    void setShowRobotInvalids(bool showPath);
    void setShowBallPlacementMarker(bool showMarker);
    void setShowDebugValueInTerminal(bool showDebug);
    void toggleSelectedRobot(rtt::world_new::view::RobotView robot);
    void setToggleFieldDirection(bool inversed);

   protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

   private:
    const rtt::world_new::World &worldManager;
    float factor{};
    int fieldmargin = Constants::WINDOW_FIELD_MARGIN();
    void drawBackground(QPainter &painter);
    void drawFieldLines(const Field &field, QPainter &painter);
    void drawFieldHints(const Field &field, QPainter &painter);

    void drawRobots(QPainter &painter, rtt::world_new::view::WorldDataView world);
    void drawRobot(QPainter &painter, rtt::world_new::view::RobotView robot, bool ourTeam, std::string role = "");
    void drawBall(QPainter &painter, rtt::world_new::view::BallView);
    void drawBallPlacementTarget(QPainter &painter);
    void drawTacticColorForRobot(QPainter &painter, rtt::world_new::view::RobotView robot);
    void drawPlusses(QPainter &painter, std::vector<Vector2> points, double width, double height);
    void drawCrosses(QPainter &painter, std::vector<Vector2> points, double width, double height);
    void drawPoints(QPainter &painter, std::vector<Vector2> points, double width, double height);
    void drawRealLifeSizedPoints(QPainter &painter, std::vector<Vector2> points, double width, double height);  // width and height are now in meters

    void drawLines(QPainter &painter, std::vector<Vector2> points);
    void drawArrows(QPainter &painter, std::vector<Vector2> points, double factor, double maxSize, bool closedArrow);
    bool shouldVisualize(Toggle toggle, int robotId);

    // utitlity functions
    std::string getTacticNameForRobot(rtt::world_new::view::RobotView robot);
    std::string getRoleNameForRobot(rtt::world_new::view::RobotView robot);
    rtt::Vector2 toScreenPosition(rtt::Vector2 fieldPos);
    rtt::Vector2 toFieldPosition(rtt::Vector2 screenPos);

    void calculateFieldSizeFactor(const Field &field);

    // interface variables
    std::vector<std::pair<std::string,
                          QColor>> tacticColors;  // map colors to tactic to visualize which robots work together
    int tacticCount = 0;                          // increases when a new tactic is used

    std::unordered_map<int, rtt::world_new::view::RobotView> selectedRobots;
    std::unordered_map<uint8_t, std::string> rolesForRobots;

    // toggles
    bool showRoles = Constants::STD_SHOW_ROLES();
    bool showTactics = Constants::STD_SHOW_TACTICS();
    bool showTacticColors = Constants::STD_SHOW_TACTICS_COLORS();
    bool showAngles = Constants::STD_SHOW_ANGLES();
    bool showVelocities = Constants::STD_SHOW_VELOCITIES();
    bool showRobotInvalids = Constants::STD_SHOW_ROBOT_INVALIDS();
    bool showBallPlacementMarker = Constants::STD_SHOW_BALL_PLACEMENT_MARKER();
    bool showDebugValueInTerminal = Constants::STD_SHOW_DEBUG_VALUES();
    bool fieldInversed = false;
    std::mutex rolesUpdate;
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_WIDGET_H
