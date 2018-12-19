#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(std::move(name), std::move(blackboard)), ioManager(false, true) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
}
bool Skill::verifyRobotCommand(roboteam_msgs::RobotCommand cmd) {
    bool output=true;
    output &= (sqrt(cmd.x_vel*cmd.x_vel+cmd.y_vel*cmd.y_vel)<=constants::MAX_VEL_CMD);
    output &= (cmd.id<=constants::MAX_ID&&cmd.id>=0);
    output &= (cmd.geneva_state>=constants::GENEVA_LEFT&&cmd.geneva_state<=constants::GENEVA_RIGHT);
    if(cmd.use_angle){
        output&= (cmd.w<=M_PI&&cmd.w>=-M_PI);
    }
    else {
        output&= (cmd.w<=constants::MAX_ANGULAR_VEL_CMD&&cmd.w>=-constants::MAX_ANGULAR_VEL_CMD);
    }
    
    return output;
}
void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    if(verifyRobotCommand(cmd)) {
        ioManager.publishRobotCommand(cmd);
    }
    else{
        ROS_ERROR_STREAM("Invalid Robot Command!");
    }
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    updateRobot();
    if (!robot) return Status::Failure;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    if (!robot) return;
    onInitialize();
}

void Skill::terminate(Status s) {
    if (!robot) return;
    onTerminate(s);
}

} // ai
} // rtt