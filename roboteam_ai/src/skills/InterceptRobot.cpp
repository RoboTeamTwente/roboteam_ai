//
// Created by rolf on 10-4-19.
//

#include "InterceptRobot.hpp"

namespace rtt{
namespace ai{
InterceptRobot::InterceptRobot(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
Skill::Status InterceptRobot::onUpdate() {
    RobotPtr robotToIntercept=world::world->getRobotForId(0,true);
    if (!robotToIntercept){
        return Status::Failure;
    }
    Vector2 robotVel=robotToIntercept->vel;
    double maxVel=2.0;
    double maxDist=1.0;
    double minDist=2.5*Constants::ROBOT_RADIUS();
    Vector2 interceptPos=robotToIntercept->pos;
    // we stand at a point in front of the robot depending on it's speed
    if (robotVel.length()<maxVel){
        interceptPos+=robotVel.stretchToLength(minDist+robotVel.length()*(maxDist-minDist));
    }
    else{
        interceptPos+=robotVel.stretchToLength(maxDist);
    }
    control::PosVelAngle velocities=gtp.getPosVelAngle(robot,interceptPos);
    command.x_vel=velocities.vel.x;
    command.y_vel=velocities.vel.y;
    command.w=velocities.angle.getAngle();
    publishRobotCommand();
    return Status::Running;
}
void InterceptRobot::onInitialize() {
    gtp.setAvoidBall(false);
    gtp.setCanMoveInDefenseArea(false);
    gtp.setCanMoveOutOfField(false);
}
void InterceptRobot::onTerminate(Skill::Status s) {

}


}
}