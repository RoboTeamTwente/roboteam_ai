//
// Created by baris on 12-12-18.
//
#include "Harass.h"

namespace rtt {
namespace ai {

void Harass::initialize() {
    robot = getRobotFromProperties(properties);
    harassBallOwner = properties->getBool("harassBallOwner");
    pickHarassmentTarget();
}

Skill::Status Harass::update() {

    if (harassmentTarget == - 1) {
        pickHarassmentTarget();
    }
    auto enemyBot = World::getRobotForId(static_cast<unsigned int>(harassmentTarget), false);
    Vector2 ballPos = World::getBall().pos;
    Vector2 targetPos;
    Vector2 enemyPos = enemyBot->pos;

    if (harassBallOwner) {
        Vector2 vec(ballPos.x - enemyBot->pos.x, ballPos.y - enemyBot->pos.y);
        targetPos = enemyPos + vec;
    }
    else {
        Vector2 i(enemyPos.x + 0.15, enemyPos.y + 0.15);
        targetPos = i;
    }


    //goToPos.goToPos(robot, targetPos, goType::basic);// TODO this might just spasm instead of going anywhere

    if (harassBallOwner && coach::doesRobotHaveBall(harassmentTarget, false)) {
        return Status::Success;
    }

    return Status::Running;
}

void Harass::pickHarassmentTarget() {
    if (harassBallOwner) {
        harassmentTarget = coach::whichRobotHasBall(false);
    }
    else {
        harassmentTarget = coach::pickHarassmentTarget(robot->id);
    }

}

}
}