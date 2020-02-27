//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_DRIBBLEBACKWARDS_H
#define ROBOTEAM_AI_DRIBBLEBACKWARDS_H

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "control/RobotCommand.h"
#include <world/Field.h>
#include <world_new/World.hpp>

namespace rtt::ai::control {
class RotateAroundBall;
class RotateWithBall;
class DribbleBackwards {
   public:
    enum BackwardsProgress : short { START, TURNING, APPROACHING, OVERSHOOTING, DRIBBLING, DRIBBLE_BACKWARDS, SUCCESS, FAIL };
    BackwardsProgress getBackwardsProgression();

   private:
    RotateAroundBall *rotateAroundBall;
    RotateWithBall *rotateAroundRobot;

    world_new::view::RobotView robot{nullptr};
    world_new::view::BallView ball{nullptr};

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
    void updateBackwardsProgress();
    RobotCommand sendBackwardsCommand(const Field &field);
    RobotCommand startTravelBackwards();
    RobotCommand sendTurnCommand();
    RobotCommand sendApproachCommand();
    RobotCommand sendOvershootCommand(const Field &field);
    RobotCommand sendDribblingCommand();
    RobotCommand sendDribbleBackwardsCommand();
    RobotCommand sendSuccessCommand();

   public:
    RobotCommand getRobotCommand(const Field &field, world_new::view::RobotView r, const Vector2 &targetP, const Angle &targetA);
    void reset();

    explicit DribbleBackwards(double errorMargin = 0.02, double angularErrorMargin = 0.02, double ballPlacementAccuracy = 0.04, double maxVel = 0.4);
    ~DribbleBackwards();
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_DRIBBLEBACKWARDS_H
