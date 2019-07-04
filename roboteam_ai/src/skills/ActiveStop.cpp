//
// Created by baris on 8-4-19.
//

#include <roboteam_ai/src/world/Field.h>
#include "ActiveStop.h"
#include "roboteam_ai/src/control/ControlUtils.h"

namespace rtt{
namespace ai {

int ActiveStop::attack = -1;

ActiveStop::ActiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}
void ActiveStop::onInitialize() {
    robot->getNumtreePosControl()->setAvoidBallDistance(0.8);

    if (attack == -1) {
        attack = robot->id;
        attacker = true;
    }
}

Skill::Status ActiveStop::onUpdate() {
    if (attacker) {
        targetPos = getOffensiveActivePoint();
    } else {
        targetPos = getDefensiveActivePoint();
    }

    if (robot->pos.dist(targetPos) > 0.3) {
        command.w = static_cast<float>((targetPos - robot->pos).angle());
    } else {
        command.w = static_cast<float>((ball->pos - robot->pos).angle());
    }

    Vector2 velocity = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos).vel;
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
    return getPoint(penaltyPos);
}

Vector2 ActiveStop::getDefensiveActivePoint() {
    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(true);
    return getPoint(penaltyPos);
}

Vector2 ActiveStop::getPoint(const Vector2 &penaltyPos) {
    Vector2 ballPos = world::world->getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(1.0); // ssl rule + significant buffer
    if (world::field->pointIsInDefenceArea(ballPos + offset)) {
        Vector2 outsideDefenseArea = Control::ControlUtils::projectPositionToOutsideDefenseArea(ballPos, 1.3);
        if ((outsideDefenseArea - ballPos).length() < 1.0) {
            return ballPos + (outsideDefenseArea-ballPos).normalize();
        }
        return outsideDefenseArea;
    }
    return ballPos + offset;
}

}
}