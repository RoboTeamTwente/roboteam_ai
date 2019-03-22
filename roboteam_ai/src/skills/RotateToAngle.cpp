//
// Created by rolf on 21/11/18.
//

#include "RotateToAngle.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {
RotateToAngle::RotateToAngle(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void RotateToAngle::onInitialize() {
    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }

    if (properties->getBool("rotateToBall")) {
        targetAngle = ((Vector2)ball->pos - robot->pos).angle();
    }

}

/// Called when the Skill is Updated
RotateToAngle::Status RotateToAngle::onUpdate() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = useAngle;
    command.w = static_cast<float>(targetAngle);
//__________________________________________________________________________________________________________
    deltaAngle = fabs(Control::constrainAngle(targetAngle - robot->angle));
    currentProgress = checkProgression();

    switch (currentProgress) {
        case ROTATING: {
            publishRobotCommand(command);
            return Status::Running;
        }
        case DONE:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }

    return Status::Failure;
}

void RotateToAngle::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = useAngle;
    command.w = targetAngle;
    publishRobotCommand(command);
}

RotateToAngle::Progression RotateToAngle::checkProgression() {
    double errorMargin = M_PI*0.03;
    if (deltaAngle > errorMargin) return ROTATING;
    else return DONE;
}

} // ai
} // rtt