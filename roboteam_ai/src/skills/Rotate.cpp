//
// Created by thijs on 24-10-18.
//

#include "Rotate.h"

namespace rtt {
namespace ai {

void Rotate::Initialize() {
    if (blackboard->HasBool("Rotate_To_Object")) {
        if (blackboard->GetBool("Rotate_To_Object")) {  // Rotate towards an object

            if (blackboard->HasInt("Rotate_Object")) {
                targetObject = blackboard->GetInt("Rotate_Object");
                bool rotateToObject = true;
            }
            else {
                ROS_ERROR("No good Rotate_Object set in BB, Rotation");
                currentProgress = Progression::FAIL;
                return;
            }

        }
        else {                                        // Rotate to an angle

            if (blackboard->HasFloat("Rotate_Angle")) {
                targetRotation = blackboard->GetFloat("Rotate_Angle");
                bool rotateToObject = false;
            }
            else {
                ROS_ERROR("No good Rotate_Angle set in BB, Rotation");
                currentProgress = Progression::FAIL;
                return;
            }

        }
    }
    else {
        ROS_ERROR("No good Rotate_To_Object set in BB, Rotation");
        currentProgress = Progression::FAIL;
        return;
    }

    if (rotateToObject) {

        Vector2 objectPos;

        switch (targetObject) {
        case 100: { // ball

            worldBall ball = World::getBall();
            objectPos.x = ball.pos.x;
            objectPos.y = ball.pos.y;

            break;
        }
        case 101: { // opponent goal
            objectPos = Field::get_their_goal_center();

            break;
        }
        case 102: { // our goal
            objectPos = Field::get_our_goal_center();


            break;
        }
        default: {  // id<50 == id our team, id>50 == (id-50) opponent team


            worldRobot otherRobot;

            if (targetObject >= 0 && targetObject < 50) {
                if (! World::getRobotForId(targetObject, true)) {
                    ROS_ERROR("Robot id invalid");
                    currentProgress = Progression::FAIL;
                    return;
                }
                otherRobot = World::getRobotForId(targetObject, true).get();
            }
            else if (targetObject < 100) {
                if (! World::getRobotForId(targetObject - 50, true)) {
                    ROS_ERROR("Robot id invalid");
                    currentProgress = Progression::FAIL;
                    return;
                }
                otherRobot = World::getRobotForId((targetObject - 50), true).get();
            }
            else {
                ROS_ERROR("Rotate_Object not known in BB, Rotation");
                currentProgress = Progression::FAIL;
                return;

            }
            objectPos = otherRobot.pos;

            break;
        }
        }
        targetRotation = objectPos.angle();

    }
}

bt::Node::Status Rotate::Update() {

    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }


    double robotAngle = robot.angle;
    double angleDifference = robotAngle - targetRotation;
    if (angleDifference < 0) angleDifference += 2*PI;

    double angularVelocity;
    double angularErrorMargin = 0.10; // within this margin, give succes.

    if (angleDifference < angularErrorMargin || angleDifference > 2*PI - angularErrorMargin) {
        return Status::Success;
    }

    if (angleDifference > PI) { angularVelocity = MAX_ANGULAR_VELOCITY; }
    else { angularVelocity = - MAX_ANGULAR_VELOCITY; }

    // Send the robotCommand.
    sendRotationCommand(angularVelocity);
    return Status::Running;
}

void Rotate::sendRotationCommand(double angularVelocity) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.w = (float) angularVelocity;
    command.dribbler = (unsigned char) false;

    publishRobotCommand(command);
}

} // ai
} // rtt
