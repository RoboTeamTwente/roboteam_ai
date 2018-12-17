//
// Created by baris on 13-12-18.
//

#include "ShootAtGoal.h"
namespace rtt {
namespace ai {

void ShootAtGoal::onInitialize() {
    robot = getRobotFromProperties(properties);
    onlyGeneva = properties->getBool("onlyGeneva");

    // onlyGeneva = properties->getBool("onlyGeneva");
    neverGeneva = properties->getBool("neverGeneva");
}

Skill::Status ShootAtGoal::onUpdate() {
    return Status::Success;
}

}
}