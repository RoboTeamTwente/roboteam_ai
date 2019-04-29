//
// Created by thijs on 29-4-19.
//

#ifndef ROBOTEAM_AI_ROBOTCOMMAND_H
#define ROBOTEAM_AI_ROBOTCOMMAND_H

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

#include "roboteam_msgs/RobotCommand.h"

namespace rtt {

class RobotCommand {
    public:
        int id = -1;
        Vector2 vel = Vector2();
        Angle angle = Angle();

        unsigned char dribbler = 0;
        unsigned char geneva = 3;
        bool kicker = false;
        double kickerVel = 0;
        bool kickerForced = false;
        bool chipper = false;
        double chipperVel = 0;
        bool chipperForced = false;

        roboteam_msgs::RobotCommand makeRobotCommand() const {
            roboteam_msgs::RobotCommand message;
            message.id = id;
            message.x_vel = vel.x;
            message.y_vel = vel.y;
            message.dribbler = dribbler;
            message.geneva_state = geneva;
            message.kicker = kicker;
            message.kicker_vel = kickerVel;
            message.kicker_forced = kickerForced;
            message.chipper = chipper;
            message.chipper_vel = chipperVel;
            message.chipper_forced = chipperForced;
            return message;
        }
};

}

#endif //ROBOTEAM_AI_ROBOTCOMMAND_H
