#include <roboteam_ai/src/control/ControlUtils.h>
#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(std::move(name), std::move(blackboard)) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
    ball = std::make_shared<roboteam_msgs::WorldBall>();

}
void Skill::publishRobotCommand() {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);

//    if(ourSideParam=="right"){
//        cmd=rotateRobotCommand(cmd);
//    }
    ioManager.publishRobotCommand(command); // We default to our robots being on the left if parameter is not set

    // refresh the robotcommand after it has been sent
    refreshRobotCommand();
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    updateRobot();
    ball = World::getBall(); // update ball position
    if (! robot) return Status::Failure;
    if (! ball) return Status::Waiting;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    ball = World::getBall();
    if (! robot) return;
    if (! ball) return;
    refreshRobotCommand();
    onInitialize();
}

void Skill::terminate(Status s) {

    std::cout << "terminating skill" << node_name() << std::endl;
    if (! robot) return;
    if (! ball) return;
    refreshRobotCommand();
    onTerminate(s);
}

roboteam_msgs::RobotCommand Skill::rotateRobotCommand(roboteam_msgs::RobotCommand &cmd) {
    roboteam_msgs::RobotCommand output = cmd;
    output.x_vel = - cmd.x_vel;
    output.y_vel = - cmd.y_vel;
    output.w = static_cast<float>(control::ControlUtils::constrainAngle(cmd.w + M_PI));
    return output;
}

void Skill::refreshRobotCommand() {
    roboteam_msgs::RobotCommand emptyCmd;
    emptyCmd.use_angle = 1;
    emptyCmd.id = robot->id;
    command = emptyCmd;
}

} // ai
} // rtt
