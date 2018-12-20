//
// Created by thijs on 18-12-18.
//

#include "goToPosInclude.h"

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
#define ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H

class ControlGoToPosBallControl {
    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Vector2 = rtt::Vector2;
        using Command = roboteam_msgs::RobotCommand;



    public:
        Command goToPos(RobotPtr robot, Vector2 &target);
};

#endif //ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
