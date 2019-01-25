#include "Skill.h"

#include "roboteam_msgs/WorldRobot.h"
#include "../utilities/Coach.h"
#include "roboteam_utils/Vector2.h"
#include <roboteam_ai/src/conditions/HasBall.hpp>
#include <roboteam_ai/src/utilities/Coach.h>
#include <roboteam_ai/src/control/ControlGoToPos.h>
#include <roboteam_ai/src/control/ControlKick.h>
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Math.h"
#include "../control/ControlUtils.h"


namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard) : bt::Leaf(std::move(name), std::move(blackboard)) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
    ball = std::make_shared<roboteam_msgs::WorldBall>();

}
void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side",ourSideParam);

    if(ourSideParam=="right"){
        cmd=rotateRobotCommand(cmd);
    }
    ioManager.publishRobotCommand(cmd); // We default to our robots being on the left if parameter is not set
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    updateRobot();
    ball = World::getBall(); // update ball position
    if (! robot) return Status::Failure;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    ball = World::getBall();
    if (!robot) return;
    onInitialize();
}

void Skill::terminate(Status s) {
    if (!robot) return;
    onTerminate(s);
}

roboteam_msgs::RobotCommand Skill::rotateRobotCommand(roboteam_msgs::RobotCommand &cmd) {
    roboteam_msgs::RobotCommand output=cmd;
    output.x_vel=-cmd.x_vel;
    output.y_vel=-cmd.y_vel;
    output.w=control::ControlUtils::constrainAngle(cmd.w+M_PI);
    return output;
}

} // ai
} // rtt