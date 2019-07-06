//
// Created by robzelluf on 4/9/19.
//

#include "ReflectKick.h"
#include "roboteam_ai/src/control/numTrees/NumTreePosControl.h"

namespace rtt {
namespace ai {

ReflectKick::ReflectKick(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void ReflectKick::onInitialize() {
    kicked = false;
    auto field = world::field->get_field();
    goalTarget = getFarSideOfGoal();
    reflectionPos = robot->pos;
    robot->getNumtreePosControl()->setAvoidBallDistance(0);
}

ReflectKick::Status ReflectKick::onUpdate() {
    // Get the angle that the robot needs to stand at, depended on the TOWARDS_GOAL_FACTOR
    robotAngle = getAngle();
    ballStartPos = ball->pos;

    if(coach::g_pass.isPassed()) {
        if(ball->vel.length() < Constants::BALL_STILL_VEL()) {
            return Status::Failure;
        }

        reflectionPos = getKicker();
        if (!ballReceiveVelSet) {
            ballReceiveVel = ball->vel;
            ballReceiveVelSet = true;
        }

        command.kicker = true;
        command.kicker_forced = robot->hasBall(Constants::MAX_KICK_RANGE() * 0.8);
        command.kicker_vel = Constants::MAX_KICK_POWER();
        intercept();
    } else {
        command.w = robotAngle;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
    }

    publishRobotCommand();
    coach::g_pass.setReadyToReceivePass(true);

    if (kicked && ballDeflected()) {
        return Status::Success;
    }
    return Status::Running;
}

// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 ReflectKick::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    Vector2 interceptPoint = reflectionPos.project(startBall, endBall);
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return interceptPoint - distanceToKicker.rotate(robot->angle);
}

void ReflectKick::intercept() {
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME() * 10;

    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = robot->getBasicPosControl()->getRobotCommand(robot, interceptPoint).vel;
    command.x_vel = velocities.x;
    command.y_vel = velocities.y;
    command.w = robotAngle;
}

void ReflectKick::onTerminate(Status s) {
    coach::g_pass.resetPass(robot->id);
    kicked = false;
}

Vector2 ReflectKick::getFarSideOfGoal() {
    Vector2 robotPos = robot->pos;
    float cornering = rtt::ai::world::field->get_field().goal_width/2.0;
    if (robotPos.y >= 0) {
        return {rtt::ai::world::field->get_their_goal_center().x,
                rtt::ai::world::field->get_their_goal_center().y + cornering};
    }
    else {
        return {rtt::ai::world::field->get_their_goal_center().x,
                rtt::ai::world::field->get_their_goal_center().y - cornering};
    }
}

Vector2 ReflectKick::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robot->pos + distanceToKicker.rotate(robot->angle);
}

double ReflectKick::getAngle() {
    Vector2 robotToGoalVector = (goalTarget - getKicker()).stretchToLength(1.0);
    Vector2 robotToBallVector = (ball->pos - getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector * TOWARDS_GOAL_FACTOR + robotToBallVector * (1 - TOWARDS_GOAL_FACTOR))).toAngle();
    return angle;
}

bool ReflectKick::ballDeflected() {
    return (ball->vel - ballReceiveVel).toAngle() > 0.01 || ball->vel.length() < 0.1;
}

}
}