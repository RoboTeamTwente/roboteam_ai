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

class NumTreePosControl : public BasicPosControl {
   private:
    using InterfaceValues = interface::Output;
    using PathPointer = std::shared_ptr<PathPoint>;

    std::vector<rtt::Vector2> triedPaths;

    world_new::view::RobotView robot{nullptr};
    const world_new::view::RobotView *magicalRobot{nullptr};
    Vector2 finalTargetPos;

    bool doRecalculatePath(world_new::view::WorldDataView *world, const Vector2 &targetPos);
    bool checkCurrentRobotCollision(world_new::view::WorldDataView *world);
    bool checkEmptyPath();
    bool checkIfTargetMoved(double maxTargetDeviation, const Vector2 &targetPos);
    bool checkIfAtEndOfPath(double maxTargetDeviation, const Vector2 &targetPos);
    bool checkIfTooFarFromCurrentPath(double maxTargetDeviation, const Vector2 &vector2);
    bool checkIfRobotWillCollideFollowingThisPath(world_new::view::WorldDataView *world);

    RobotCommand computeCommand(const Vector2 &exactTargetPos);

    // constants
    static constexpr double MAX_CALCULATION_TIME = 10.0;            // Max calculation time in ms
    double DT = 0.1;                                                // timestep for ODE model
    static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 0.25;  // 3x robot radius

    // collisions
    Collision getCollision(world_new::view::WorldDataView *world, const PathPointer &point, double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS);
    Collision getRobotCollision(const PathPointer &point, const std::vector<world_new::view::RobotView> &robots, double distance);
    Collision getBallCollision(const PathPointer &point, const world_new::view::BallView &ball);
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
    world_new::view::WorldDataView *world;
    const world::Field *field = nullptr;

   private:
    bool allowIllegalPositions = false;
    Vector2 currentlyAvoidingDefenseAreaPosition;
    bool currentlyAvoidingDefenseArea = false;
    double currentMaxRobotVel = 0;

    // new paths
    PathPointer computeNewPoint(const std::shared_ptr<PathPoint> &oldPoint, const Vector2 &subTarget);
    std::pair<std::vector<Vector2>, PathPointer> getNewTargets(const PathPointer &collisionPoint, const Collision &collision);

    // paths
    void tracePath(world_new::view::WorldDataView *world);
    std::vector<PathPoint> backTrackPath(PathPointer point, const PathPointer &root);
    double remainingStraightLinePathLength(const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos);
    std::vector<PathPoint> path;
    bool pathHasRobotCollision = false;

    void checkInterfacePID() override;

   public:
    NumTreePosControl() = default;
    explicit NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

    void clear();

    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robotPtr, const Vector2 &targetPos) override;
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robotPtr, const Vector2 &targetPos,
                                 bool illegalPositions);
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robotPtr, const Vector2 &targetPos,
                                 const Angle &targetAngle) override;
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robotPtr, const Vector2 &targetPos,
                                 const Angle &targetAngle, bool illegalPositions);

    bool checkChangeInMaxRobotVel();
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_NUMTREEPOSCONTROL_H
