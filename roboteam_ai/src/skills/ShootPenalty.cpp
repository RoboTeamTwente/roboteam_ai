//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"
#include "../world/World.h"
#include "../world/Ball.h"
#include "../world/Robot.h"

namespace rtt {
namespace ai {

ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void ShootPenalty::onInitialize() {
    tick = 0;

    genevaSet = false;
    genevaState = 5;
    gtp.setCanMoveInDefenseArea(true);
    gtp.setAutoListenToInterface(false); //HACK HACK
    gtp.setCanMoveOutOfField(false);
    gtp.updatePid({1.0,0.0,0.2});
    lineP=8.0;
    additionalBallDist=Vector2(0.05,0.0);
    forcedKickOn=true;
    forcedKickRange=Constants::MAX_KICK_RANGE()-0.01;
}
bt::Node::Status ShootPenalty::onUpdate() {
    if (! robot) return Status::Running;
    if (! genevaSet) {
        genevaState = determineGenevaState();
        genevaSet = true;
    }
    double gain=ydiff*lineP;
    if (tick < genevaChangeTicks&&ydiff<0.03) {
    double gain=ydiff*P;
    if (tick < genevaChangeTicks&&ydiff<0.01) {
        tick ++;
        command.x_vel=0;
        command.y_vel=gain;
        command.w=0;
        command.geneva_state = genevaState;
        auto ball=world::world->getBall();
        if (ball){
            ballPos=ball->pos;
        }
    }
    else {

        if (ball&&!world::field->pointIsInDefenceArea(ballPos,false,-0.1)){
            Vector2 targetPos=world::world->getBall()->pos+additionalBallDist;
            if (world::field->pointIsInDefenceArea(ballPos,false,0.2)){
                auto cmd=gtp.getRobotCommand(robot,targetPos);
                command.x_vel=cmd.vel.x;
                command.y_vel=cmd.vel.y+gain;
            }
            else{
                auto cmd=robot->getNumtreePosControl()->getRobotCommand(robot,targetPos);
                command.x_vel=cmd.vel.x;
                command.y_vel=cmd.vel.y;
            }
            command.w = 0;
            command.kicker = true;
            command.kicker_vel = Constants::MAX_KICK_POWER();
            if (forcedKickOn||!robot->hasWorkingBallSensor()){
                double dist=robot->calculateDistanceToBall(ballPos);
                if (dist!=-1.0){
                    command.kicker_forced= dist<forcedKickRange;
                } else {
                    std::cout << "HELP!" << std::endl;
                }
            }
            command.geneva_state = genevaState;
        }

    }
    command.geneva_state = genevaState;
    std::cout<<"robotID "<<robot->id<<" "<<genevaState<<std::endl;
    publishRobotCommand();
    return Status::Running;
}

int ShootPenalty::determineGenevaState() {
    // determine the shortest position from where to kick the ball
    if (robot->pos.y<ball->pos.y){
        return genevaState=1;
    }
    return genevaState=5;
}

}
}