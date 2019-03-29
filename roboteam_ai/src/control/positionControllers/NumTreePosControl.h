//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_NUMTREEPOSCONTROL_H
#define ROBOTEAM_AI_NUMTREEPOSCONTROL_H

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "PosVelAngle.h"
#include "PosController.h"
#include "PathPoint.h"
#include "ForcePosControl.h"

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl : public ForcePosControl {
private:
    const double MAX_CALCULATION_TIME = 5.0;   // Max time in ms

    using InterfaceValues = interface::InterfaceValues;

    const double dt = 0.07;
    const double defaultRobotCollisionRadius = 3*Constants::ROBOT_RADIUS_MAX();
    int robotID = - 1;
    Vector2 pos;
    Vector2 vel;
    bool avoidBall = false;
    bool canGoOutsideField = true;

    void drawCross(Vector2 &pos, QColor color = Qt::green);
    void drawPoint(Vector2 &pos, QColor color = Qt::green);
    void addDataInInterface(std::vector<std::pair<rtt::Vector2, QColor>> displayColorData);
    void redrawInInterface();

    std::vector<std::pair<Vector2, QColor>> displayData;
    bool doRecalculatePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
    double remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos);

    PosVelAngle computeCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot);

    std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> getNewTargets(
            std::shared_ptr<PathPoint> collisionPoint);
    bool checkCollision(std::shared_ptr<PathPoint> point, double collisionRadius = 0.27);
    std::shared_ptr<PathPoint> computeNewPoint(std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget);
    void tracePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
    std::vector<PathPoint> backTrackPath(std::shared_ptr<PathPoint> point, std::shared_ptr<PathPoint> root);
    Vector2 findCollisionPos(std::shared_ptr<PathPoint> point, double collisionRadius = 0.27);

    std::vector<PathPoint> path;
public:
    explicit NumTreePosControl() = default;
    void clear();
    Vector2 finalTargetPos;
    PosVelAngle getPosVelAngle(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 &targetPos) override;
    void setCanGoOutsideField(bool _canGoOutsideField);

};

}
}
}

#endif //ROBOTEAM_AI_NUMTREEPOSCONTROL_H
