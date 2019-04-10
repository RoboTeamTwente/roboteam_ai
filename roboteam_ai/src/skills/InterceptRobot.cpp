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
    RobotPtr robotToIntercept=world::world->getRobotForId(1,true);
    if (!robotToIntercept){
        return Status::Failure;
    }
    Vector2 robotVel=robotToIntercept->vel;
    double maxVel=2.0;
    double maxDist=1.0;
    double minDist=2.5*Constants::ROBOT_RADIUS();
    Vector2 interceptPos=robotToIntercept->pos;
    // we stand at a point in front of the robot depending on it's speed
    Vector2 angle=robotToIntercept->angle.toVector2(1.0);
    if (robotVel.length()<maxVel){
        interceptPos+=angle.stretchToLength(minDist+robotVel.length()*(maxDist-minDist));
    }
    else{
        interceptPos+=angle.stretchToLength(maxDist);
    }
    control::PosVelAngle velocities=gtp.getPosVelAngle(robot,interceptPos);
    Vector2 XYvel=control::ControlUtils::velocityLimiter(velocities.vel,4.0);
    command.x_vel=XYvel.x;
    command.y_vel=XYvel.y;
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