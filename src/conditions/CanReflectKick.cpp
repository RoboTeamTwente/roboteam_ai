//
// Created by robzelluf on 6/14/19.
//

#include <world/World.h>
#include <world/Field.h>
#include "conditions/CanReflectKick.h"
#include "world/Robot.h"
#include "world/Ball.h"
#include "skills/ReflectKick.h"
#include "control/ControlUtils.h"

namespace rtt {
namespace ai {

CanReflectKick::CanReflectKick(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status CanReflectKick::onUpdate() {
    if (false) {
        return Status::Failure;
    }

    if (!control::ControlUtils::objectVelocityAimedToPoint(ball->getPos(), ball->vel, robot->pos, 0.6)) {
        return Status::Failure;
    }

    if (ball->vel.length() < 0.5) {
        return Status::Failure;
    }

    Angle ballToRobotAngle = (robot->pos - ball->getPos()).toAngle();
    Angle robotToGoalAngle = (world::field->get_their_goal_center() - robot->pos).toAngle();

    // If both angles are either positive or negative, reflectKick will not work (robot cannot get behind the ball properly)
    if (ballToRobotAngle * robotToGoalAngle > 0) {
        return Status::Failure;
    }

    Angle receiveAngle = getApproximateReflectAngle();

    Angle angleDifference = abs(robotToGoalAngle.angleDiff(receiveAngle));
    if(angleDifference.getAngle() < MAX_BALL_RECEIVE_ANGLE) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

double CanReflectKick::getApproximateReflectAngle() {
    Vector2 goalTarget = world::field->get_their_goal_center();

    Vector2 robotToGoalVector = (goalTarget - robot->getKicker()).stretchToLength(1.0);
    Vector2 robotToBallVector = (ball->getPos() - robot->getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector + robotToBallVector) * 0.7).toAngle();
    return angle;
}

}
}