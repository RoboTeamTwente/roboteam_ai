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
}

ReflectKick::Status ReflectKick::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) return Status::Failure;

    angleToGoalTarget = (goalTarget - robot->pos).toAngle();
    angleToBall = (ball->pos - robot->pos).toAngle();

    if(!coach::g_pass.isPassed()) {
        command.w = angleToGoalTarget + ((angleToBall - angleToGoalTarget) / 2);
    } else {
        if (world::world->ourRobotHasBall(robot->id)) {
            command.kicker = 1;
            command.kicker_forced = 1;
        } else {
            intercept();
        }
    }

    publishRobotCommand();
    coach::g_pass.setReadyToReceivePass(true);

    return Status::Running;
}

// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 ReflectKick::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    return Vector2(robot->pos).project(startBall, endBall);
}

void ReflectKick::intercept() {
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;
    velocities = control::ControlUtils::velocityLimiter(velocities);
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = angleToGoalTarget + ((angleToBall - angleToGoalTarget) / 2);
    command.dribbler = 1;

    publishRobotCommand();
}

void ReflectKick::onTerminate(Status s) {}

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

}
}