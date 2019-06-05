//
// Created by baris on 12-12-18.
//
#include "Harass.h"

namespace rtt {
namespace ai {

Harass::Harass(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Harass::onInitialize() {
    harassBallOwner = properties->getBool("harassBallOwner");
    pickHarassmentTarget();
}

Skill::Status Harass::onUpdate() {

    updateRobot();
    if (! robot) {
        return Status::Failure;
    }
    if (harassmentTarget == - 1) {
        pickHarassmentTarget();
    }
    auto enemyBot = world::world->getRobotForId(static_cast<unsigned int>(harassmentTarget), false);

    Vector2 ballPos = ball->pos;
    Vector2 targetPos;
    Vector2 enemyPos = enemyBot->pos;

    if (harassBallOwner) {
        Vector2 vec = {ballPos - enemyPos};
        targetPos = ballPos + vec;
    }
    else {
        Vector2 i(enemyPos.x + 0.35, enemyPos.y + 0.35);
        targetPos = i;
    }

    std::cout << "call gotopos with target pos" << targetPos << std::endl;
    std::cout << "call gotopos with robot pos           " << robot->pos << std::endl;

    goToPos.getRobotCommand(robot, targetPos);

    if (harassBallOwner && !world::world->theirRobotHasBall(harassmentTarget)) {
        return Status::Success;
    }
    // TODO make something that will make harassment stop if something happens else we assume that there is a tree
    //  change because some sort of special event happened
    return Status::Running;
}

void Harass::pickHarassmentTarget() {
//    if (harassBallOwner) {
//        harassmentTarget = Coach::whichRobotHasBall(false);
//    }
//    else {
//        harassmentTarget = Coach::pickHarassmentTarget(robot->id);
//    }

    harassmentTarget = 0;

}

} // ai
} // rtt