//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

void Pass::onInitialize() {
    ballPlacement = properties->getBool("BallPlacement");
    determineRobotToPassTo();
    numTreeGtp.setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;
    robotToPassTo = world::world->getRobotForId(static_cast<unsigned int>(robotToPassToID), true);

    bool isBehindBall = coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.30, robotToPassTo->pos, robot->pos);
    auto behindBallPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.30, getKicker());
    bool isOnLineToBall = control::ControlUtils::distanceToLine(robot->pos, ball->pos, behindBallPos) < 0.0255;
    bool hasBall = world::world->ourRobotHasBall(robot->id, Constants::MAX_BALL_RANGE());

    bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.8;
    bool ballIsShotTowardsReceiver = control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel, robotToPassTo->pos);

    if (ballIsMovingFast && ballIsShotTowardsReceiver) {
        coach::g_pass.setPassed(true);
        return Status::Success;
    } else if (isOnLineToBall && isBehindBall) {
        return hasBall ? shoot() : getBall();
    }
    return moveBehindBall(behindBallPos);
}

/// determine which robot we should pass towards.
void Pass::determineRobotToPassTo() {
    robotToPassToID = ballPlacement ? coach::g_pass.getRobotBeingPassedTo() : coach::g_pass.initiatePass(robot->id);
}

/// this is the method we call when we are far from the desired position
bt::Leaf::Status Pass::moveBehindBall(Vector2 behindBallPos) {
    targetPos = behindBallPos;

    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL());
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);

    if (targetPos.dist(robot->pos) < 0.5) {
        command.w = static_cast<float>( (Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>( (targetPos - robot->pos).angle());
    }

    publishRobotCommand();

    return bt::Leaf::Status::Running;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
bt::Leaf::Status Pass::getBall() {
    targetPos = ball->pos;
    control::PosVelAngle pva = basicGtp.getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL(), 0.3);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>((getKicker() - robot->pos).angle());

    publishRobotCommand();
    return bt::Leaf::Status::Running;
}

/// Now we should have the ball and kick it.
bt::Leaf::Status Pass::shoot() {
    if (coach::g_pass.isReadyToReceivePass()) {
        numTreeGtp.setAvoidBall(false);
        targetPos = getKicker();
        control::PosVelAngle pva = basicGtp.getPosVelAngle(robot, targetPos);
        pva.vel = control::ControlUtils::velocityLimiter(pva.vel, 0.1);
        command.x_vel = static_cast<float>(pva.vel.x);
        command.y_vel = static_cast<float>(pva.vel.y);
        command.w = static_cast<float>((getKicker() - robot->pos).angle());

        command.kicker_forced = 1;
        command.kicker_vel = determineKickForce((Vector2(ball->pos) - getKicker()). length());

        publishRobotCommand();
    }
    return Status::Running;
}

/// Determine how fast we should kick for a pass at a given distance
double Pass::determineKickForce(double distance) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    // take square root of distance and scale it vertically such that the max kick force and max distance for max kick force are correct.
    double kickSpeed = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist) ;
    return static_cast<float>(kickSpeed);
}
Vector2 Pass::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robotToPassTo->pos + distanceToKicker.rotate(robotToPassTo->angle);
}

} // ai
} // rtt

