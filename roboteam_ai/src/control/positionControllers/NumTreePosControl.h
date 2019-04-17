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

class Collision;
class PathPoint;

class NumTreePosControl : public ForcePosControl {
    public:

    private:
        using InterfaceValues = interface::InterfaceValues;
        using PathPointer = std::shared_ptr<PathPoint>;
        Robot robot = {};
        Vector2 finalTargetPos;

        bool doRecalculatePath(const Vector2 &targetPos);
        PosVelAngle computeCommand();

        // constants
        const double MAX_CALCULATION_TIME = 5.0;         // Max calculation time in ms
        const double DT = 0.07;                          // timestep for ODE model
        static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 0.28; // 3x robot radius

        // interface functions
        void drawCross(Vector2 &pos, const QColor &color = Qt::green);
        void drawPoint(Vector2 &pos, QColor color = Qt::green);
        void addDataInInterface(std::vector<std::pair<rtt::Vector2, QColor>> displayColorData);
        void redrawInInterface();
        std::vector<std::pair<Vector2, QColor>> displayData;

        // collisions
        Collision getCollision(const PathPointer &point, double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS);
        Collision getRobotCollision(const Vector2 &collisionPos, const std::vector<Robot> &robots, double distance);

        // new paths
        PathPointer computeNewPoint(const std::shared_ptr<PathPoint> &oldPoint, const Vector2 &subTarget);
        std::pair<std::vector<Vector2>, PathPointer> getNewTargets(
                const PathPointer &collisionPoint, const Collision &collision);

        // paths
        void tracePath();
        std::vector<PathPoint> backTrackPath(PathPointer point, const PathPointer &root);
        double remainingStraightLinePathLength(
                const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos);
        std::vector<PathPoint> path;

        void checkInterfacePID() override;

    public:
        explicit NumTreePosControl() = default;
        explicit NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

        void clear();
        PosVelAngle getPosVelAngle(const RobotPtr &robot, Vector2 &targetPos) override;
};

}
}
}

#endif //ROBOTEAM_AI_NUMTREEPOSCONTROL_H
