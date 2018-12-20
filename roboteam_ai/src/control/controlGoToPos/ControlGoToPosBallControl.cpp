//
// Created by thijs on 18-12-18.
//

#include "ControlGoToPosBallControl.h"
ControlGoToPosBallControl::Command ControlGoToPosBallControl::goToPos(ControlGoToPosBallControl::RobotPtr robot,
        Vector2 &target) {

    Command command;
    command.id = robot->id;
    return command;
}
