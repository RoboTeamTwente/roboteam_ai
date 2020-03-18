//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_DRIBBLEBACKWARDS_H
#define ROBOTEAM_AI_DRIBBLEBACKWARDS_H

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "control/RobotCommand.h"
#include "world/Field.h"

namespace rtt::world_new::view{
    class RobotView;
}

namespace rtt::ai::control {
using namespace rtt::ai::world;
class RotateAroundBall;
class RotateWithBall;
class DribbleBackwards {
   public:
    enum BackwardsProgress : short { START, TURNING, APPROACHING, OVERSHOOTING, DRIBBLING, DRIBBLE_BACKWARDS, SUCCESS, FAIL };
    BackwardsProgress getBackwardsProgression();

   private:
    RotateAroundBall *rotateAroundBall;
    RotateWithBall *rotateAroundRobot;

    BackwardsProgress backwardsProgress = START;
    void printBackwardsProgress();

    // variables for backwards progress
    Vector2 approachPosition;
    std::pair<Vector2, Vector2> backwardsDribbleLine;
    Angle lockedAngle;
    Angle targetAngle;
    Angle finalTargetAngle;
    Vector2 targetPos;
    Vector2 finalTargetPos;

    // error margins and accuracy
    int waitingTicks;
    bool failedOnce;

    double errorMargin;
    double angleErrorMargin = 10.0 / 180.0 * M_PI;
    double ballPlacementAccuracy;
    double maxVel;

   public:
    void setMaxVel(double maxVel);

   private:
    // functions for backwards progress
    void updateBackwardsProgress(world_new::view::RobotView _robot);
    RobotCommand sendBackwardsCommand(const Field &field, world_new::view::RobotView _robot);
    RobotCommand startTravelBackwards(world_new::view::RobotView _robot);
    RobotCommand sendTurnCommand(world_new::view::RobotView _robot);
    RobotCommand sendApproachCommand(world_new::view::RobotView _robot);
    RobotCommand sendOvershootCommand(const Field &field, world_new::view::RobotView _robot);
    RobotCommand sendDribbleBackwardsCommand(world_new::view::RobotView _robot);
    RobotCommand sendDribblingCommand();
    RobotCommand sendSuccessCommand();

   public:
    RobotCommand getRobotCommand(world_new::view::RobotView _robot, const Vector2 &targetP, const Angle &targetA);
    void reset();

    explicit DribbleBackwards(double errorMargin = 0.02, double angularErrorMargin = 0.02, double ballPlacementAccuracy = 0.04, double maxVel = 0.4);
    ~DribbleBackwards();
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_DRIBBLEBACKWARDS_H
