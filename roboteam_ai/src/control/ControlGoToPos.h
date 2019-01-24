//
// Created by thijs on 10-12-18.
//



#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H


#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/io/IOManager.h>
#include <utility>

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

namespace rtt {
namespace ai {
namespace control {

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

class ControlGoToPos {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Command = roboteam_msgs::RobotCommand;

        Vector2 goToPosBallControl(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosBallControl gtpBallControl;

        Vector2 goToPosLuTh(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosLuTh gtpLuth;

        Vector2 goToPosLowLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosLowLevel gtpLowlevel;

        Vector2 goToPosHighLevel(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosHighLevel gtpHighlevel;

        Vector2 goToPosBezier(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBezier gtpBezier;

        Vector2 goToPosForce(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosForce gtpForce;

        Vector2 goToPosBasic(RobotPtr robot, Vector2 &targetPos);
        //ControlGoToPosBasic gtpBasic;

        double errorMargin = 0.3;
        double distanceToTarget(RobotPtr robot, Vector2 &targetPos);

        Controller pid = Controller(3.0, 0.0, 0.5);
    public:
        ControlGoToPos();

        void clear(GoToType goToType);
        Vector2 goToPos(RobotPtr robot, Vector2 &position);
        Vector2 goToPos(RobotPtr robot, Vector2 &position, GoToType goToType);

        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
