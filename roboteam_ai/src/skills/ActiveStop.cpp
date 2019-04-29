//
// Created by baris on 8-4-19.
//

#include "ActiveStop.h"
namespace rtt{
namespace ai {

int ActiveStop::attack = -1;

ActiveStop::ActiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}
void ActiveStop::onInitialize() {
    goToPos.setAvoidBall(0.8);
    if (attack == -1) {
        attack = robot->id;
        attacker = true;
    }
}

Skill::Status ActiveStop::onUpdate() {
    if (attacker)
        targetPos = getOffensiveActivePoint();
    else
        targetPos = getDefensiveActivePoint();

    command.w = static_cast<float>((targetPos - robot->pos).angle());
    Vector2 velocityRaw = goToPos.getPosVelAngle(robot, targetPos).vel;
    Vector2 velocity = control::ControlUtils::velocityLimiter(velocityRaw, 1.2);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand();
    return Status::Running;


}

void ActiveStop::onTerminate(Skill::Status s) {
    attack = -1;
}

Vector2 ActiveStop::getOffensiveActivePoint() {

    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(false);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.666); // ssl rule + some buffer + rtt spirit
    if (rtt::ai::world::field->pointIsInDefenceArea(ballPos + offset)){
        return ((ballPos+offset).rotate(M_PI).stretchToLength(2));
    }
    return ballPos + offset;

}

Vector2 ActiveStop::getDefensiveActivePoint() {

    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(true);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.666); // ssl rule + some buffer + rtt spirit
    if (rtt::ai::world::field->pointIsInDefenceArea(ballPos + offset)){
        return (((ballPos+offset).rotate(M_PI)).stretchToLength(2));
    }
    return ballPos + offset;

}

}
}