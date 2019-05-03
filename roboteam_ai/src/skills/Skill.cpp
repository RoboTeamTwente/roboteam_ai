#include <roboteam_ai/src/control/ControlUtils.h>
#include "Skill.h"
#include "../utilities/RobotDealer.h"


namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(std::move(name), std::move(blackboard)) {
    robot = std::make_shared<Robot>(Robot());
    ball = std::make_shared<Ball>(Ball());
}

void Skill::publishRobotCommand() {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);
    Vector2 vel(command.x_vel,command.y_vel);
    if (vel.length()==0){
        command.x_vel=0;
        command.y_vel=0;
    }
    else{
        vel=control::ControlUtils::velocityLimiter(vel,2.0,0.2);
        command.x_vel=vel.x;
        command.y_vel=vel.y;
    }

    if (command.kicker != 0) {
        std::cerr << "Kicker   " << node_name() << std::endl;
    }
    if(Constants::GRSIM() && ourSideParam=="right"){
      command=rotateRobotCommand(command);
    }

    if (command.id == -1) {
        if (robot && robot->id != -1) {
            command.id = robot->id;
            ioManager.publishRobotCommand(command); // We default to our robots being on the left if parameter is not set

        }
    } else {
        ioManager.publishRobotCommand(command); // We default to our robots being on the left if parameter is not set

    }
    // refresh the robotcommand after it has been sent
    refreshRobotCommand();
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    std::string roleName = properties->getString("ROLE");
    robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
    updateRobot();
    ball = world::world->getBall(); // update ball position
    if (! robot || robot->id == -1) return Status::Failure;
    if (! ball) return Status::Waiting;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    ball = world::world->getBall();
    if (! robot || robot->id == -1) return;
    if (! ball) return;
    refreshRobotCommand();
    onInitialize();
}

void Skill::terminate(Status s) {
    if (!init) {
        return;
    }
    init = false;
    if (! robot || robot->id == -1) return;
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
    emptyCmd.id = robot ? robot->id : -1;
    emptyCmd.geneva_state = 3;
    command = emptyCmd;
}

} // ai
} // rtt
