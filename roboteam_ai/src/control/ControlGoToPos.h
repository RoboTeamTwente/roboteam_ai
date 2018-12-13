//
// Created by thijs on 10-12-18.
//

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/io/IOManager.h>

#include "ros/ros.h"
#include "../io/IOManager.h"
#include "../../src/control/ControlUtils.h"
#include "../utilities/Constants.h"
#include "../skills/Skill.h"

#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H

namespace control {

class ControlGoToPos {
    public:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;

        enum GoToType {
          noPreference,
          basic,
          lowLevel,
          highLevel,
          force,
          luTh,
          bezier,
        };

        static void goToPos(RobotPtr robot, rtt::Vector2 &position);
        static void goToPos(RobotPtr robot, rtt::Vector2 &position, GoToType goToType);
    private:
        static void goToPosLuTh(RobotPtr robot, rtt::Vector2 &targetPos);
        static void goToPosLowLevel(RobotPtr robot, rtt::Vector2 &targetPos);
        static void goToPosHighLevel(RobotPtr robot, rtt::Vector2 &targetPos);
        static void goToPosBezier(RobotPtr robot, rtt::Vector2 &targetPos);
        static void goToPosForce(RobotPtr robot, rtt::Vector2 &targetPos);
        static void goToPosBasic(RobotPtr robot, rtt::Vector2 &targetPos);
        static void publishRobotCommand(roboteam_msgs::RobotCommand &command);

};

} // control

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
