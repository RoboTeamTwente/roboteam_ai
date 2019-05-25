//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_NUMTREEPOSCONTROL_H
#define ROBOTEAM_AI_NUMTREEPOSCONTROL_H

#include <roboteam_ai/src/interface/api/Output.h>
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
    private:
        using InterfaceValues = interface::Output;
        using PathPointer = std::shared_ptr<PathPoint>;

        std::vector<rtt::Vector2> triedPaths;
        RobotPtr robot;
        Vector2 finalTargetPos;

        bool doRecalculatePath(const Vector2 &targetPos);
        PosVelAngle computeCommand(const Vector2 &exactTargetPos);

        // constants
        const double MAX_CALCULATION_TIME = 25.0;         // Max calculation time in ms
        double DT = 0.1;                          // timestep for ODE model
        static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 0.25; // 3x robot radius

        // collisions
        Collision getCollision(const PathPointer &point, double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS);
        Collision getRobotCollision(const Vector2 &collisionPos, const std::vector<RobotPtr> &robots, double distance);

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
        NumTreePosControl() = default;
        ~NumTreePosControl() override = default;
        explicit NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

        void clear();
        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) override;
        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos) override;

};

}
}
}

#endif //ROBOTEAM_AI_NUMTREEPOSCONTROL_H
