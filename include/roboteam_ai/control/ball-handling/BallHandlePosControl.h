//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <control/pid.h>
#include <roboteam_utils/LineSegment.h>
#include <roboteam_utils/Vector2.h>
#include <utilities/Constants.h>
#include <world/Ball.h>
#include <world/Robot.h>

#include "control/RobotCommand.h"
#include "control/numtrees/NumTreePosControl.h"
namespace rtt::ai::control {

class DribbleBackwards;
class DribbleForwards;
class RotateAroundBall;
class RotateWithBall;
class BallHandlePosControl : public NumTreePosControl {
   private:
    DribbleForwards *dribbleForwards;
    DribbleBackwards *dribbleBackwards;
    RotateWithBall *rotateWithBall;
    RotateAroundBall *rotateAroundBall;

    double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.2;
    double maxBackwardsVelocity = Constants::GRSIM() ? 0.4 : 0.5;
    double ballPlacementAccuracy = 0.12;

    constexpr static double ERROR_MARGIN = 0.02;
    constexpr static double ANGLE_ERROR_MARGIN = 0.010 * M_PI;
    constexpr static double MAX_BALL_DISTANCE = Constants::ROBOT_RADIUS() * 2.0;
    constexpr static double MIN_VEL_FOR_MOVING_BALL = 0.3162277660168;
    constexpr static double TARGET_BALL_DISTANCE = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    constexpr static double ROBOT_IS_TOUCHING_BALL = TARGET_BALL_DISTANCE * 1.05;

//    RobotPtr robot;
//    BallPtr ball;
    world_new::view::RobotView robot;
    world_new::view::BallView ball;
    Vector2 targetPos;
    Angle targetAngle;
    Angle lockedAngle = 0;
    int ticksNotMoving = 0;

    pidfVals pidfGoToBall = std::make_tuple(0.0, 0.0, 0.0, 1.5);
    PID xGoToBallPID = PID(pidfGoToBall);
    PID yGoToBallPID = PID(pidfGoToBall);

    pidfVals pidfBallHandle = std::make_tuple(0.1, 0.0, 0.0, 0.8);
    PID xBallHandlePID = PID(pidfBallHandle);
    PID yBallHandlePID = PID(pidfBallHandle);

    void updatePID(pidVals newPID);

   public:
    RobotCommand controlWithPID(PID &xpid, PID &ypid, const RobotCommand &robotCommand);

    enum TravelStrategy : short { FORWARDS, BACKWARDS, NO_PREFERENCE };

   private:
    TravelStrategy preferredTravelStrategy = NO_PREFERENCE;

    // general functions
    RobotCommand handleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool shouldGoToBall, bool ballIsFarFromTarget = true);
    RobotCommand goToBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget);

    // get status of ball handling
   public:
    enum Status : short { GET_BALL, HANDLING_BALL, FINALIZING, SUCCESS, FAILURE };
    Status getStatus();

   private:
    Status status = GET_BALL;
    void printStatus();

   public:
    explicit BallHandlePosControl(bool canMoveInDefenseArea = true);
    ~BallHandlePosControl();
    void setMaxVelocity(double maxV);
    void setMaxForwardsVelocity(double maxV);
    void setMaxBackwardsVelocity(double maxV);
//    RobotCommand getRobotCommand(world::World *world, const Field *field, const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) override;
//    RobotCommand getRobotCommand(world::World *world, const Field *field, const RobotPtr &r, const Vector2 &targetP, const Angle &targetA, TravelStrategy travelStrategy);
//    RobotCommand getRobotCommand(world::World *world, const Field *field, const RobotPtr &r, const Vector2 &targetP) override;

    // TODO: Implement these:
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const Field *field, const world_new::view::RobotView &r, const Vector2 &targetP, const Angle &targetA);
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const Field *field, const world_new::view::RobotView &r, const Vector2 &targetP, const Angle &targetA,
                                 TravelStrategy travelStrategy);

   private:
    bool isCrashingIntoOpponentRobot(const LineSegment &driveLine);
    bool isCrashingOutsideField(const LineSegment &driveLine);

    RobotCommand goToMovingBall();
    RobotCommand goToIdleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget);
    RobotCommand finalizeBallHandle();
    RobotCommand interceptMovingBall(const Vector2 &projectionPosition, double ballToProjectionDistance, const Angle &robotAngleTowardsBallVel);
    RobotCommand goBehindBall(const Vector2 &ballStillPosition);
    RobotCommand interceptMovingBallTowardsBall();
    Vector2 movingBallTowardsBallTarget;
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
