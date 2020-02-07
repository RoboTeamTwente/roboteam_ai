//
// Created by robzelluf on 4/9/19.
//

#include "skills/ReflectKick.h"
#include "control/numtrees/NumTreePosControl.h"
#include "world/FieldComputations.h"

namespace rtt::ai {

ReflectKick::ReflectKick(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void ReflectKick::onInitialize() {
    kicked = false;
    goalTarget = (*field).getTheirGoalCenter();
    reflectionPos = robot->pos;
    robot->getNumtreePosControl()->setAvoidBallDistance(0);
}

ReflectKick::Status ReflectKick::onUpdate() {
    // Get the angle that the robot needs to stand at, depended on the TOWARDS_GOAL_FACTOR
    robotAngle = getAngle();
    ballStartPos = ball->getPos();

    if (coach::g_pass.isPassed()) {
        if (ball->getVel().length() < Constants::BALL_STILL_VEL()) {
            return Status::Failure;
        }

        reflectionPos = getKicker();
        if (!ballReceiveVelSet) {
            ballReceiveVel = ball->getVel();
            ballReceiveVelSet = true;
        }

        command.set_kicker(true);
        command.set_chip_kick_forced(robot->hasBall(Constants::MAX_KICK_RANGE() * 0.8));
        command.set_chip_kick_vel(Constants::MAX_KICK_POWER());
        intercept();
    } else {
        command.set_w(robotAngle);
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(0);
    }

    publishRobotCommand();
    coach::g_pass.setReadyToReceivePass(true);

    if (kicked && ballDeflected()) {
        return Status::Success;
    }
    return Status::Running;
}

// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 ReflectKick::computeInterceptPoint(const Vector2 &startBall, const Vector2 &endBall) {
    Vector2 interceptPoint = reflectionPos.project(startBall, endBall);
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return interceptPoint - distanceToKicker.rotate(robot->angle);
}

void ReflectKick::intercept() {
    ballStartVel = ball->getVel();
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME() * 10;

    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = robot->getBasicPosControl()->getRobotCommand(world, field, robot, interceptPoint).vel;
    command.mutable_vel()->set_x(velocities.x);
    command.mutable_vel()->set_y(velocities.y);
    command.set_w(robotAngle);
}

void ReflectKick::onTerminate(Status s) {
    coach::g_pass.resetPass(robot->id);
    kicked = false;
}

Vector2 ReflectKick::getFarSideOfGoal() {
    Vector2 robotPos = robot->pos;
    float cornering = (*field).getGoalWidth() / 2.0;
    if (robotPos.y >= 0) {
        return {(*field).getTheirGoalCenter().x, (*field).getTheirGoalCenter().y + cornering};
    } else {
        return {(*field).getTheirGoalCenter().x, (*field).getTheirGoalCenter().y - cornering};
    }
}

Vector2 ReflectKick::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robot->pos + distanceToKicker.rotate(robot->angle);
}

double ReflectKick::getAngle() {
    Vector2 robotToGoalVector = (goalTarget - getKicker()).stretchToLength(1.0);
    Vector2 robotToBallVector = (ball->getPos() - getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector * TOWARDS_GOAL_FACTOR + robotToBallVector * (1 - TOWARDS_GOAL_FACTOR))).toAngle();
    return angle;
}

bool ReflectKick::ballDeflected() { return (ball->getVel() - ballReceiveVel).toAngle() > 0.01 || ball->getVel().length() < 0.1; }

}  // namespace rtt::ai