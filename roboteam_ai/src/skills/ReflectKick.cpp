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
}

ReflectKick::Status ReflectKick::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) return Status::Failure;

    angleToGoalTarget = (goalTarget - getKicker()).toAngle();
    angleToBall = (ball->pos - getKicker()).toAngle();
    command.w = robotAngle;

    ballStartPos = ball->pos;

    if(!coach::g_pass.isPassed()) {
        robotAngle = (angleToGoalTarget + ((angleToBall - angleToGoalTarget) / 2)).getAngle();
        reflectionPos = getKicker();
    } else {
        if (world::world->ourRobotHasBall(robot->id)) {
            command.kicker = 1;
            command.kicker_forced = 1;
            kicked = true;
            std::cout << "KICK!" << std::endl;
        } else {
            intercept();
        }
    }

    publishRobotCommand();
    coach::g_pass.setReadyToReceivePass(true);

    if (kicked) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 ReflectKick::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    Vector2 interceptPoint = reflectionPos.project(startBall, endBall);
    Vector2 distanceToKicker = {Constants::DISTANCE_TO_KICKER(), 0};
    return interceptPoint - distanceToKicker.rotate(robot->angle);
}

void ReflectKick::intercept() {
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;
    velocities = control::ControlUtils::velocityLimiter(velocities);
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = robotAngle;
    command.dribbler = 1;
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

Vector2 ReflectKick::getKicker() {
    Vector2 distanceToKicker = {Constants::DISTANCE_TO_KICKER(), 0};
    return robot->pos + distanceToKicker.rotate(robot->angle);
}

}
}