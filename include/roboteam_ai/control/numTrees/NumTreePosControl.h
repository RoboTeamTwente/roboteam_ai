//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_NUMTREEPOSCONTROL_H
#define ROBOTEAM_AI_NUMTREEPOSCONTROL_H

#include <interface/api/Output.h>
#include "control/BasicPosControl.h"
#include "PathPoint.h"
#include "Collision.h"

namespace rtt {
namespace ai {
namespace control {

class PathPoint;
class Collision;
class NumTreePosControl : public BasicPosControl {
    private:
        using InterfaceValues = interface::Output;
        using PathPointer = std::shared_ptr<PathPoint>;

        std::vector<rtt::Vector2> triedPaths;
        RobotPtr robot;
        Vector2 finalTargetPos;

        bool doRecalculatePath(const Vector2 &targetPos);
        bool checkCurrentRobotCollision();
        bool checkEmptyPath();
        bool checkIfTargetMoved(double maxTargetDeviation, const Vector2 &targetPos);
        bool checkIfAtEndOfPath(double maxTargetDeviation, const Vector2 &targetPos);
        bool checkIfTooFarFromCurrentPath(double maxTargetDeviation, const Vector2 &vector2);
        bool checkIfRobotWillCollideFollowingThisPath();

        RobotCommand computeCommand(const Vector2 &exactTargetPos);

        // constants
        static constexpr double MAX_CALCULATION_TIME = 10.0;         // Max calculation time in ms
        /** Why is this under constants?...*/ double DT = 0.1;                          // timestep for ODE model
        static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 0.25; // 3x robot radius

        // collisions
        Collision getCollision(const PathPointer &point, double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS);
        Collision getRobotCollision(const PathPointer &point, const std::vector<RobotPtr> &robots, double distance);
        Collision getBallCollision(const PathPointer &point, const BallPtr &ball);
        Collision getFieldCollision(const PathPointer &point);
        Collision getDefenseAreaCollision(const PathPointer &point);
        Collision getGoalCollision(const PathPointer &point);
        Collision getBallPlacementCollision(const PathPointer &point);

        Collision currentCollisionWithRobot;
        Collision currentCollisionWithFinalTarget;
    public:
        const Collision &getCurrentCollisionWithRobot() const;
        const Collision &getCurrentCollisionWithFinalTarget() const;
protected:
    world::World * world = nullptr;
    world::Field * field = nullptr;
    private:

        bool allowIllegalPositions = false;
        Vector2 currentlyAvoidingDefenseAreaPosition;
        bool currentlyAvoidingDefenseArea = false;
        double currentMaxRobotVel = 0;

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
        bool pathHasRobotCollision = false;

        void checkInterfacePID() override;

    public:
        NumTreePosControl() = default;
        explicit NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

        void clear();

        RobotCommand getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr, const Vector2 &targetPos) override;
        RobotCommand getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr, const Vector2 &targetPos, bool illegalPositions);
        RobotCommand getRobotCommand(world::World * world,  world::Field * field, const RobotPtr &robotPtr, const Vector2 &targetPos,
                const Angle &targetAngle) override;
        RobotCommand getRobotCommand(world::World * world,  world::Field * field, const RobotPtr &robotPtr, const Vector2 &targetPos, const Angle &targetAngle,
                bool illegalPositions);

        bool checkChangeInMaxRobotVel();
};

}
}
}

#endif //ROBOTEAM_AI_NUMTREEPOSCONTROL_H
