//
// Created by robzelluf on 6/14/19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Field.h>
#include "CanReflectKick.h"
#include "../world/Robot.h"
#include "../skills/ReflectKick.h"

namespace rtt {
namespace ai {

CanReflectKick::CanReflectKick(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {};

bt::Node::Status CanReflectKick::onUpdate() {
    if(!robot->hasWorkingBallSensor()) {
        return Status::Failure;
    }

    Angle ballToRobotAngle = (robot->pos - ball->pos).toAngle();
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
    Vector2 robotToBallVector = (ball->pos - robot->getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector + robotToBallVector) * 0.5).toAngle();
    return angle;
}

}
}