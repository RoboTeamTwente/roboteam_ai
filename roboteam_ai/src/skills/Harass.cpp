//
// Created by baris on 12-12-18.
//
#include "Harass.h"

namespace rtt {
namespace ai {

Harass::Harass(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

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
    auto enemyBot = World::getRobotForId(static_cast<unsigned int>(harassmentTarget), false);
    Vector2 ballPos = World::getBall().pos;
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

    goToPos.goToPos(robot, targetPos, goToType::basic);

    if (harassBallOwner && ! coach::doesRobotHaveBall(harassmentTarget, false)) {
        return Status::Success;
    }
    // TODO make something that will make harassment stop if something happens else we assume that there is a tree
    //  change because some sort of special event happened
    return Status::Running;
}

void Harass::pickHarassmentTarget() {
//    if (harassBallOwner) {
//        harassmentTarget = coach::whichRobotHasBall(false);
//    }
//    else {
//        harassmentTarget = coach::pickHarassmentTarget(robot->id);
//    }

    harassmentTarget = 0;

}

} // ai
} // rtt