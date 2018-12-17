//
// Created by robzelluf on 12/17/18.
//

#ifndef ROBOTEAM_AI_CONTROLKICK_H
#define ROBOTEAM_AI_CONTROLKICK_H


#include <roboteam_msgs/RobotCommand.h>

namespace control {

class ControlKick {
public:
    ControlKick();
private:
    using Command = roboteam_msgs::RobotCommand;
    using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
    float MAX_KICKER_VEL = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
    rtt::ai::io::IOManager ioManager;

    void kick(RobotPtr& robot);
    void kick(RobotPtr& robot, unsigned char kicker_forced);
    void kick(RobotPtr& robot, unsigned char kicker_forced, float kicker_vel);
    void publishRobotCommand(roboteam_msgs::RobotCommand &command);
};

}

#endif //ROBOTEAM_AI_CONTROLKICK_H
