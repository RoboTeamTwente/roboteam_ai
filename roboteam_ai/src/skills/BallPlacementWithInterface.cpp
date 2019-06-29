//
// Created by thijs on 15-5-19.
//

#include "BallPlacementWithInterface.h"
#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

void BallPlacementWithInterface::onUpdate() {
    robot = world::world->getUs()[0];
    if (!robot) return;

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto robotCommand = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle, control::BallHandlePosControl::TravelStrategy::FORWARDS );

    if (targetPos == previousTargetPos &&
        ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = robotCommand.angle;
        command.dribbler = 0;
        publishRobotCommand();
        return;
    }
    command.x_vel = static_cast<float>(robotCommand.vel.x);
    command.y_vel = static_cast<float>(robotCommand.vel.y);
    command.w = robotCommand.angle;
    command.dribbler = robotCommand.dribbler;
    publishRobotCommand();

    previousTargetPos = targetPos;
    return;
}

void BallPlacementWithInterface::publishRobotCommand() {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);

    limitRobotCommand();

    if (std::isnan(command.x_vel) || std::isnan(command.y_vel)) {
        std::cout << "ERROR: x or y vel in command is NAN in Skill " << node_name().c_str() << "!" << "  robot  " << robot->id << std::endl;
    }

    // Make sure both kicker and chipper vel are set, so that it works for both GrSim and Serial
    if(command.kicker_vel > command.chipper_vel) {
        command.chipper_vel = command.kicker_vel;
    } else {
        command.kicker_vel = command.chipper_vel;
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

void BallPlacementWithInterface::limitRobotCommand() {
    auto limitedVel = Vector2(command.x_vel, command.y_vel);
    limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
    limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w);
    robot->setPidPreviousVel(limitedVel);
    if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
        std::cout << "ERROR: ROBOT WILL HAVE NAN~!?!?!KLJ#Q@?LK@ " << node_name().c_str() << "!" << "  robot  " << robot->id << std::endl;
        robot->setPidPreviousVel(robot->vel);
    }
    command.x_vel = limitedVel.x;
    command.y_vel = limitedVel.y;
}

void BallPlacementWithInterface::refreshRobotCommand() {
    roboteam_msgs::RobotCommand emptyCmd;
    emptyCmd.use_angle = 1;
    emptyCmd.id = robot ? robot->id : -1;
    emptyCmd.geneva_state = 0;
    command = emptyCmd;
}

std::string BallPlacementWithInterface::node_name() {
    return "TC";
}

}
}