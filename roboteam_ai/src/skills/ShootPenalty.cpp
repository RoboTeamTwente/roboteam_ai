//
// Created by baris on 11-3-19.
//

#include <roboteam_ai/src/interface/api/Input.h>
#include "ShootPenalty.h"

namespace rtt {
namespace ai {

void ShootPenalty::onInitialize() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 robotPos = robot->pos;
    targetPos = ballPos + (robotPos - ballPos).rotate(fakeOffset.getAngle());
    aim();

}

Skill::Status ShootPenalty::onUpdate() {

    if (isPenaltyShot()) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.geneva_state = 3;
        command.dribbler = 0;
        return Status::Success;
    }


    robot->getShotController()->makeCommand(robot->getShotController()->getShotData(*robot,
            rtt::ai::world::field->get_their_goal_center(), false, control::BallSpeed::MAX_SPEED, true,
            control::ShotPrecision::HIGH), command);

    publishRobotCommand();
    return Status::Running;



}

void ShootPenalty::onTerminate(Skill::Status s) {
    // clean up the coach or whereever logic you use

}
ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
bool ShootPenalty::isPenaltyShot() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    if ((ballPos - rtt::ai::world::field->getPenaltyPoint(false)).length() > 0.13){
        shot = true;
        return true;
    }
    return false;

}
void ShootPenalty::aim() {

    // TODO: make better and smarter by looking at the keeper ant etc.
    Vector2 goal = rtt::ai::world::field->get_their_goal_center();
    double goalHalfLength = (goal - rtt::ai::world::field->getPenaltyPoint(false)).length()/2.0;
    aimPoint = {goal.x, goal.y+(goalHalfLength*(6.0/7.0))};

}

}
}