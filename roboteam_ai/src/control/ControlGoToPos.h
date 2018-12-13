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

//  ______________________
//  |                    |
//  |  INCLUDE GoToPos   |
//  |____________________|
//

#include "controlGoToPos/ControlGoToPosLuTh.h"


#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H

namespace control {

class ControlGoToPos {
    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Vector2 = rtt::Vector2;
        using Command = roboteam_msgs::RobotCommand;


        void goToPosLuTh(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosLuTh luth;

        void goToPosLowLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosLowLevel lowlevel;

        void goToPosHighLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosHighLevel highlevel;

        void goToPosBezier(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBezier bezier;

        void goToPosForce(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosForce force;

        void goToPosBasic(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBasic basic;

        void publishRobotCommand(Command &command);
        double errorMargin = 0.3;
        double distanceToTarget(RobotPtr robot, Vector2 &targetPos);

    public:

        enum GoToType {
          noPreference,
          basic,
          lowLevel,
          highLevel,
          force,
          luTh,
          bezier,
        };

        void goToPos(RobotPtr robot, Vector2 &position);
        void goToPos(RobotPtr robot, Vector2 &position, GoToType goToType);

};

} // control

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
