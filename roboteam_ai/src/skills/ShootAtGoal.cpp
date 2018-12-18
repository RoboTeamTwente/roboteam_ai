//
// Created by baris on 13-12-18.
//

#include "ShootAtGoal.h"
namespace rtt {
namespace ai {

void ShootAtGoal::onInitialize() {
    robot = getRobotFromProperties(properties);
}

Skill::Status ShootAtGoal::onUpdate() {
    updateRobot();
    switch(currentProgress) {
        case ROTATING: {
            // TODO: Add in functions
            // rotateWithBall
            // OR
            // getBehindBall
        }
        case READY: {
            controlKick.kick(robot);
            currentProgress = DONE;
        }
        case DONE: return Status::Success;
        case TURN_GENEVA:break;
    }

    return Status::Success;
}

}
}