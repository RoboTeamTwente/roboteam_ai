//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"
namespace rtt{
namespace ai {


void ShootPenalty::onInitialize() {
    Vector2 ballPos = rtt::ai::World::getBall()->pos;
    Vector2 robotPos = robot -> pos;
    targetPos = ballPos + (robotPos - ballPos).rotate(fakeOffset.getAngle());
}



Skill::Status ShootPenalty::onUpdate() {

    switch (progress) {

        case GOING: {
            Vector2 deltaPos = (targetPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = READY;
            }
            else{
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                command.w = static_cast<float>((targetPos-robot->pos).angle());
                command.geneva_state = 1;
                Vector2 velocity = goToPos.goToPos(robot, targetPos, control::PosControlType::NUMERIC_TREES).vel;
                publishRobotCommand(command);
            }
            break;
        }

        case ROTATING:{
            if ((robot->w - fakeOffset) > errorMarginAng) {
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                command.geneva_state = 1;
                command.w = static_cast<float>(fakeOffset);
                publishRobotCommand(command);
            }
            else {
                progress = READY;
                break;
            }

            break;}

        case READY:{
            //TODO: make rule checks
            roboteam_msgs::RobotCommand command;
            command.id = robot->id;
            command.geneva_state = 1;
            publishRobotCommand(command);
            progress = SHOOTING;
            break;}


        case SHOOTING:{

            Vector2 ballPos = rtt::ai::World::getBall()->pos;
            roboteam_msgs::RobotCommand command;
            command.id = robot->id;
            command.use_angle = 1;
            command.w = static_cast<float>((ballPos-robot->pos).angle());
            command.geneva_state = 1;
            command.kicker = static_cast<unsigned char>(true);
            command.kicker_vel = Constants::MAX_KICK_POWER();
            Vector2 velocity = goToPos.goToPos(robot, ballPos, control::PosControlType::BASIC).vel;
            publishRobotCommand(command);
            break;}


        case AVOIDING:{
            break;}


    }


    return Status::Failure;
}



void ShootPenalty::onTerminate(Skill::Status s) {
    // clean up the coach or whereever logic you use

}
ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

}
}