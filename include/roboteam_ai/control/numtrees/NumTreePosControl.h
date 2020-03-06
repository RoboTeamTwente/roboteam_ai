//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_NUMTREEPOSCONTROL_H
#define ROBOTEAM_AI_NUMTREEPOSCONTROL_H

#include <interface/api/Output.h>

#include "Collision.h"
#include "PathPoint.h"
#include "control/BasicPosControl.h"

namespace rtt::ai::control {

class PathPoint;
class Collision;
class NumTreePosControl : public BasicPosControl {
   private:
    using InterfaceValues = interface::Output;
    using PathPointer = std::shared_ptr<PathPoint>;

    std::vector<rtt::Vector2> triedPaths;
    Vector2 finalTargetPos;

    // constants
    static constexpr double MAX_CALCULATION_TIME = 10.0;            // Max calculation time in ms
    double DT = 0.1;                                                // timestep for ODE model
    static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 0.25;  // 3x robot radius

    // collisions
    Collision getFieldCollision(world_new::view::RobotView _robot, const PathPointer &point);
    Collision getGoalCollision(const PathPointer &point);
    Collision getBallPlacementCollision(const PathPointer &point, world_new::view::BallView ball);

    Collision currentCollisionWithRobot;
    Collision currentCollisionWithFinalTarget;

   public:
    [[nodiscard]] const Collision &getCurrentCollisionWithRobot() const;
    [[nodiscard]] const Collision &getCurrentCollisionWithFinalTarget() const;

   private:
    bool allowIllegalPositions = false;
    Vector2 currentlyAvoidingDefenseAreaPosition;
    bool currentlyAvoidingDefenseArea = false;
    double currentMaxRobotVel = 0;

    // new paths
    PathPointer computeNewPoint(const std::shared_ptr<PathPoint> &oldPoint, const Vector2 &subTarget);
    std::pair<std::vector<Vector2>, PathPointer> getNewTargets(const PathPointer &collisionPoint, const Collision &collision);

    // paths
    std::vector<PathPoint> backTrackPath(PathPointer point, const PathPointer &root);
    double remainingStraightLinePathLength(const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos);
    std::vector<PathPoint> path;
    bool pathHasRobotCollision = false;

    void checkInterfacePID() override;

   public:
    NumTreePosControl() = default;
    explicit NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

    void clear();

    RobotCommand getRobotCommand(int robotId, const Vector2 &targetPos) override;
    RobotCommand getRobotCommand(int robotId, const Vector2 &targetPos, const Angle &targetAngle) override;

    bool doRecalculatePath(const Vector2 &targetPos, world_new::view::RobotView _robot);
    void tracePath(world_new::view::RobotView _robot);

    Collision getRobotCollision(world_new::view::RobotView _robot, const PathPointer &point, const std::vector<world_new::view::RobotView> &robots, double distance);
    Collision getCollision(world_new::view::RobotView _robot, const PathPointer &point, double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS);
    Collision getBallCollision(const PathPointer &point, world_new::view::BallView ball);
    Collision getDefenseAreaCollision(world_new::view::RobotView _robot, const PathPointer &point);

    RobotCommand computeCommand(world_new::view::RobotView _robot, const Vector2 &exactTargetPos);

    bool checkChangeInMaxRobotVel();
    bool checkCurrentRobotCollision(world_new::view::RobotView _robot);
    bool checkEmptyPath(world_new::view::RobotView _robot);
    bool checkIfTargetMoved(world_new::view::RobotView _robot, double maxTargetDeviation, const Vector2 &targetPos);
    bool checkIfAtEndOfPath(world_new::view::RobotView _robot, double maxTargetDeviation, const Vector2 &targetPos);
    bool checkIfTooFarFromCurrentPath(world_new::view::RobotView _robot, double maxTargetDeviation, const Vector2 &vector2);
    bool checkIfRobotWillCollideFollowingThisPath(world_new::view::RobotView _robot);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_NUMTREEPOSCONTROL_H
