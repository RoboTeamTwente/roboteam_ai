//
// Created by thijs on 24-10-18.
//

#include "Rotate.h"


namespace rtt {
    namespace ai {

        void Rotate::Initialize() {
        }

        bt::Node::Status Rotate::Update() {

            // Get kickVelocity from blackboard, otherwise it is a default value.
            roboteam_msgs::WorldBall ball = World::getBall();

            Vector2 relativeBallPos;
            relativeBallPos.x = ball.pos.x - robot.pos.x;
            relativeBallPos.y = ball.pos.y - robot.pos.y;

            double ballAngle = relativeBallPos.angle();
            double robotAngle = robot.angle;
            double desiredAngle = blackboard->HasDouble("robotAngle") ? blackboard->GetDouble("robotAngle") : ballAngle;
            double angleDifference = robotAngle - desiredAngle;
            if (angleDifference < 0) angleDifference += 2 * PI;
            // -pi, pi
            // -pi, pi
            double angularVelocity;


            if (angleDifference > PI) {
                angularVelocity = MAX_ANGULAR_VELOCITY*angleDifference / 2*PI;
            } else if (angleDifference > -PI) {
                angularVelocity = MAX_ANGULAR_VELOCITY*angleDifference;
            } else {
                //TODO: ask if you need to return succes/when it stops executing
                //TODO: is this the right way?
            }

            // Send the robotCommand.
            sendRotationCommand(angularVelocity);


            return Status::Running;
        }

        void Rotate::sendRotationCommand(double angularVelocity){
            roboteam_msgs::RobotCommand command;
            command.id = robot.id;
            command.w = (float) angularVelocity;
            command.dribbler = (unsigned char) false;

            io::robotCommandPublisher.publish(command);
        }



    } // ai
} // rtt
