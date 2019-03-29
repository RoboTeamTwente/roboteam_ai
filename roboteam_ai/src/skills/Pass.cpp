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
    numTreeGtp.setAvoidBall(true);
    forceGtp.setAvoidBall(false);
    basicGtp.setAvoidBall(false);
    robotToPassToID = coach::g_ballPlacement.initiatePass();
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;
    robotToPassTo = World::getRobotForId(static_cast<unsigned int>(robotToPassToID), true);

    auto behindBallpos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.30, robotToPassTo->pos);


   // bool isBehindBall = coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.35, robotToPassTo->pos, robot->pos, 0.05);
    bool isAimed = control::ControlUtils::distanceToLine(robot->pos, ball->pos, behindBallpos) < 0.02;

    bool hasBall = World::ourBotHasBall(robot->id, Constants::MAX_BALL_RANGE());
    bool ballIsMovingFast = Vector2(World::getBall()->vel).length() > 0.4;

    if (ballIsMovingFast) {
        coach::g_ballPlacement.setRobotBeingPassedTo(-1);
        coach::g_ballPlacement.setPassed(true);
        return Status::Success;
    } else if (isAimed) {
        return hasBall ? shoot() : getBall();
    }
    return moveBehindBall();
}

/// this is the method we call when we are far from the desired position
bt::Leaf::Status Pass::moveBehindBall() {
    std::cout << "Getting behind ball" << std::endl;
    targetPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.30, robotToPassTo->pos);
    sendMoveCommand();
    return bt::Leaf::Status::Running;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
bt::Leaf::Status Pass::getBall() {
    std::cout << "Getting ball" << std::endl;

    targetPos = ball->pos;
    control::PosVelAngle pva = basicGtp.getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL(), 0.3);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>( (Vector2(robotToPassTo->pos) - robot->pos).angle());

    publishRobotCommand();
    return bt::Leaf::Status::Running;
}

// Now we should have the ball and kick it.
bt::Leaf::Status Pass::shoot() {
    std::cout << "Kicking" << std::endl;


        targetPos = robotToPassTo->pos;
        control::PosVelAngle pva = basicGtp.getPosVelAngle(robot, targetPos);
        pva.vel = control::ControlUtils::velocityLimiter(pva.vel, 0.1);
        command.x_vel = static_cast<float>(pva.vel.x);
        command.y_vel = static_cast<float>(pva.vel.y);
        command.w = static_cast<float>( (Vector2(robotToPassTo->pos) - robot->pos).angle());

    command.kicker_forced = 1;
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
    double distance = (Vector2(ball->pos) - robotToPassTo->pos).length();
    double kicker_vel_multiplier = distance > maxPowerDist ? 1.0 : distance / maxPowerDist;
    command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER()*kicker_vel_multiplier);
    publishRobotCommand();
    return Status::Running;
}

/// send a command to move the current robot to targetPos with a certain goToType.
void Pass::sendMoveCommand(const double minimumSpeed) {
    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL(), minimumSpeed);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>( (targetPos - robot->pos).angle());
    publishRobotCommand();
}

} // ai
} // rtt

