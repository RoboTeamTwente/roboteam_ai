//
// Created by robzelluf on 4/9/19.
//

#include "ReflectKick.h"

namespace rtt {
namespace ai {

ReflectKick::ReflectKick(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void ReflectKick::onInitialize() {
    auto field = world::field->get_field();
    goalTarget = getFarSideOfGoal();
    reflectionPos = robot->pos;
    robot->getNumtreeGtp()->setAvoidBall(0);
}

ReflectKick::Status ReflectKick::onUpdate() {
    // Get the angle between the kicker of the robot and the long corner of the goal (furthest corner from the robot)
    angleToGoalTarget = (goalTarget - getKicker()).toAngle();
    angleToBall = (ball->pos - getKicker()).toAngle();

    // Get the angle that the robot needs to stand at, depended on the TOWARDS_GOAL_FACTOR
    robotAngle = getAngle();
    command.w = robotAngle;
    ballStartPos = ball->pos;

    if(!coach::g_pass.isPassed()) {
        reflectionPos = getKicker();
    } else {
        if (willHaveBall() || kicked) {
            if(!ballReceiveVelSet) {
                ballReceiveVel = ball->vel;
                ballReceiveVelSet = true;
            }
            Vector2 velocity = {2, 0};
            velocity = velocity.rotate(robot->angle);

            command.x_vel = velocity.x;
            command.y_vel = velocity.y;
            command.kicker = 1;
            command.kicker_forced = 1;
            kicked = true;
            kickTicks++;
        } else {
            intercept();
        }
    }

    publishRobotCommand();
    coach::g_pass.setReadyToReceivePass(true);

    if (kicked && (ballDeflected() || kickTicks >= MAX_KICK_TICKS)) {
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
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();

    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = robot->getNumtreeGtp()->getPosVelAngle(robot, interceptPoint).vel;
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = robotAngle;
}

void ReflectKick::onTerminate(Status s) {
    kickTicks=0;
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
    return (angleToGoalTarget + ((angleToBall - angleToGoalTarget) * (1 - TOWARDS_GOAL_FACTOR))).getAngle();
}

bool ReflectKick::willHaveBall() {
    Vector2 futureBallPos = ball->pos + ball->vel * SECONDS_AHEAD;
    double ballDistance = robot->calculateDistanceToBall(futureBallPos);
    return ballDistance < Constants::MAX_KICK_RANGE() && ballDistance >= 0;
}

bool ReflectKick::ballDeflected() {
    return (ball->vel - ballReceiveVel).toAngle() > 0.01 || ball->vel.length() < 0.1;
}

}
}