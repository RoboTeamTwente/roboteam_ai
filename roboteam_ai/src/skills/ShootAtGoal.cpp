//
// Created by baris on 13-12-18.
//

#include "ShootAtGoal.h"
namespace rtt {
namespace ai {

void ShootAtGoal::initialize() {
    robot = getRobotFromProperties(properties);
}


Skill::Status ShootAtGoal::update() {
    return Status::Success;
}


void ShootAtGoal::terminate(Skill::Status s) {
    Skill::terminate(s);
}
}
}