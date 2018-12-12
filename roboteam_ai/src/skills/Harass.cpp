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