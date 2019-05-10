//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"

namespace rtt {
namespace ai {

void ShootPenalty::onInitialize() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 robotPos = robot->pos;
    targetPos = ballPos + (robotPos - ballPos).rotate(fakeOffset.getAngle());
    shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::HIGH, control::BallSpeed::MAX_SPEED, true);
    aim();

}

Skill::Status ShootPenalty::onUpdate() {

    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 deltaPos = (ballPos - robot->pos);

    if (deltaPos.length() >= errorMarginPos) {
        command.w = static_cast<float>((ballPos - robot->pos).angle());
        command.geneva_state = 1;
        Vector2 velocity = goToPos.getPosVelAngle(robot, ballPos).vel;
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        publishRobotCommand();
        return Status::Running;

    }
    else{
        shotControl->makeCommand(shotControl->getShotData(*robot, aimPoint), command);
        publishRobotCommand();
    }
    if (isPenaltyShot()) {
        publishRobotCommand();
        return Status::Success;

    }
    return Status::Failure;


}

void ShootPenalty::onTerminate(Skill::Status s) {
    // clean up the coach or whereever logic you use

}
ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
bool ShootPenalty::isPenaltyShot() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    if ((ballPos - rtt::ai::world::field->getPenaltyPoint(false)).length() > 0.30){
        shot = true;
        return true;
    }
    return false;

}
void ShootPenalty::aim() {

    // TODO: make better and smarter by looking at the keeper ant etc.
    Vector2 goal = rtt::ai::world::field->get_their_goal_center();
    double goalHalfLength = (goal - rtt::ai::world::field->getPenaltyPoint(false)).length()/2.0;
    aimPoint = {goal.x, goal.y+goalHalfLength};

}

}
}