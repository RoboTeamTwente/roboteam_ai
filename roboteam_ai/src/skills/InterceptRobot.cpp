//
// Created by rolf on 10-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Robot.h>
#include "InterceptRobot.hpp"
#include <roboteam_ai/src/control/BasicPosControl.h>

namespace rtt{
namespace ai{

InterceptRobot::InterceptRobot(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

void InterceptRobot::onInitialize() {
//    gtp.setAvoidBallDistance(false);
//    gtp.setCanMoveInDefenseArea(false);
//    gtp.setCanMoveOutOfField(false);
}

Skill::Status InterceptRobot::onUpdate() {
    //TODO: fix that the robotToIntercept is called from coach
    RobotPtr robotToIntercept=world::world->getRobotForId(1,true);
    if (!robotToIntercept){
        return Status::Failure;
    }
    Vector2 interceptPos = getInterceptPos(*robotToIntercept);
    auto velocities = robot->getBasicPosControl()->getRobotCommand(robot, interceptPos);
    command.x_vel=velocities.vel.x;
    command.y_vel=velocities.vel.y;
    command.w=velocities.angle.getAngle();

    publishRobotCommand();
    return Status::Running;
}

Vector2 InterceptRobot::getInterceptPos(Robot robotToIntercept) {
    Vector2 robotVel=robotToIntercept.vel;
    double maxVel=2.0;//velocity at which or above it we drive at the max distance in front of the robot
    double maxDist=1.0; //max distance in front of the robot
    double minDist=2.5*Constants::ROBOT_RADIUS();//minimum distance in front of the robot
    Vector2 interceptPos=robotToIntercept.pos;
    // we stand at a point in front of the robot depending on it's speed
    Vector2 angle=robotToIntercept.angle.toVector2(1.0);
    if (robotVel.length()<maxVel){
        interceptPos+=angle.stretchToLength(minDist+robotVel.length()*(maxDist-minDist));
    }
    else{
        interceptPos+=angle.stretchToLength(maxDist);
    }
    return interceptPos;
}



}
}