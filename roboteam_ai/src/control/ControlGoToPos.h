//
// Created by thijs on 10-12-18.
//



#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H


#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/io/IOManager.h>

#include "ros/ros.h"
#include "../io/IOManager.h"
#include "../../src/control/ControlUtils.h"
#include "../../src/control/Controller.h"
#include "../utilities/Constants.h"

//  ______________________
//  |                    |
//  |  INCLUDE GoToPos   |
//  |____________________|
//

#include "controlGoToPos/ControlGoToPosLuTh.h"
#include "controlGoToPos/ControlGoToPosBallControl.h"

namespace control {

class ControlGoToPos {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Vector2 = rtt::Vector2;
        using Command = roboteam_msgs::RobotCommand;
        rtt::ai::io::IOManager ioManager;

        void goToPosBallControl(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosBallControl gtpBallcontrol;

        void goToPosLuTh(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosLuTh gtpLuth;

        void goToPosLowLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosLowLevel gtpLowlevel;

        void goToPosHighLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosHighLevel gtpHighlevel;

        void goToPosBezier(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBezier gtpBezier;

        void goToPosForce(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosForce gtpForce;

        void goToPosBasic(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBasic gtpBasic;
        Controller pidPos;
        //ControlGoToPosBasic basic;

        void publishRobotCommand(Command &command);
        double errorMargin = 0.3;
        double distanceToTarget(RobotPtr robot, Vector2 &targetPos);

    public:
        ControlGoToPos();

        enum GoToType {
          noPreference,
          ballControl,
          basic,
          lowLevel,
          highLevel,
          force,
          luTh,
          bezier,
        };

        void clear(GoToType goToType);
        void goToPos(RobotPtr robot, Vector2 &position);
        void goToPos(RobotPtr robot, Vector2 &position, GoToType goToType);

};

} // control

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
