//
// Created by robzelluf on 12/17/18.
//

#ifndef ROBOTEAM_AI_CONTROLROTATE_H
#define ROBOTEAM_AI_CONTROLROTATE_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/io/IOManager.h>

#include "ros/ros.h"
#include "../io/IOManager.h"
#include "../../src/control/ControlUtils.h"
#include "../utilities/Constants.h"

namespace control {

class ControlRotate {
    public:
        ControlRotate();

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        void rotateToBall();
        void rotateToRobot();
};

}

#endif //ROBOTEAM_AI_CONTROLROTATE_H
