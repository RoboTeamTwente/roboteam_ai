//
// Created by robzelluf on 6/14/19.
//

#include "conditions/CanReflectKick.h"
#include <world/Field.h>
#include <world/World.h>
#include "control/ControlUtils.h"
#include "skills/ReflectKick.h"
#include "world/Ball.h"
#include "world/Robot.h"

namespace rtt::ai {

CanReflectKick::CanReflectKick(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status CanReflectKick::onUpdate() {
    if (!control::ControlUtils::objectVelocityAimedToPoint(ball->getPos(), ball->getVel(), robot->pos, 0.6)) {
        return Status::Failure;
    }

    if (ball->getVel().length() < 0.5) {
        return Status::Failure;
    }

    Angle ballToRobotAngle = (robot->pos - ball->getPos()).toAngle();
    Angle robotToGoalAngle = (field->get_field().get(THEIR_GOAL_CENTER) - robot->pos).toAngle();

    // If both angles are either positive or negative, reflectKick will not work (robot cannot get behind the ball properly)
    if (ballToRobotAngle * robotToGoalAngle > 0) {
        return Status::Failure;
    }

    Angle receiveAngle = getApproximateReflectAngle();

    Angle angleDifference = abs(robotToGoalAngle.angleDiff(receiveAngle));
    if (angleDifference.getAngle() < MAX_BALL_RECEIVE_ANGLE) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

double CanReflectKick::getApproximateReflectAngle() {
    Vector2 goalTarget = field->get_field().get(THEIR_GOAL_CENTER);

    Vector2 robotToGoalVector = (goalTarget - robot->getKicker()).stretchToLength(1.0);
    Vector2 robotToBallVector = (ball->getPos() - robot->getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector + robotToBallVector) * 0.7).toAngle();
    return angle;
}

}  // namespace rtt::ai