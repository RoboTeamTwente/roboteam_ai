/*
 * returns SUCCESS if reflect kick is an option
 * otherwise FAILURE
 */

#include "conditions/CanReflectKick.h"

namespace rtt::ai {

CanReflectKick::CanReflectKick(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status CanReflectKick::onUpdate() {
    if (!control::ControlUtils::objectVelocityAimedToPoint(ball->get()->getPos(), ball->get()->getVelocity(), robot->get()->getPos(), 0.6)) {
        return Status::Failure;
    }

    if (ball->get()->getVelocity().length() < 0.5) {
        return Status::Failure;
    }

    Angle ballToRobotAngle = (robot->get()->getPos() - ball->get()->getPos()).toAngle();
    Angle robotToGoalAngle = ((*field).getTheirGoalCenter() - robot->get()->getPos()).toAngle();

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
    Vector2 goalTarget = (*field).getTheirGoalCenter();

    Vector2 robotToGoalVector = (goalTarget - robot->getKicker()).stretchToLength(1.0);
    Vector2 robotToBallVector = (ball->get()->getPos() - robot->getKicker()).stretchToLength(1.0);
    Angle angle = ((robotToGoalVector + robotToBallVector) * 0.7).toAngle();
    return angle;
}

}  // namespace rtt::ai