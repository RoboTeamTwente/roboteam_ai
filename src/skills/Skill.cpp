#include "skills/Skill.h"
#include "utilities/RobotDealer.h"
#include "utilities/Constants.h"
#include "utilities/GameStateManager.hpp"
#include "control/ControlUtils.h"
#include "world/Robot.h"
#include "world/World.h"
#include "world/Ball.h"
#include <cmath>

namespace rtt::ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(std::move(name), std::move(blackboard)) {
    robot = std::make_shared<Robot>(Robot());
    ball = std::make_shared<Ball>(Ball());
}

void Skill::publishRobotCommand() {
    if(!::rtt::world::settings::Settings::settings->isLeft()){
      command=rotateRobotCommand(command);
    }

    limitRobotCommand();

    if (std::isnan(command.vel().x()) || std::isnan(command.vel().y())) {
        std::cout << "ERROR: x or y vel in command is NAN in Skill " << node_name().c_str() << "!" << "  robot  " << robot->id << std::endl;
    }
    
    if (command.id() == -1) {
        if (robot && robot->id != -1) {
            command.set_id(robot->id);
            io::io.publishRobotCommand(command); // We default to our robots being on the left if parameter is not set
        }
    } else {
        io::io.publishRobotCommand(command); // We default to our robots being on the left if parameter is not set
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
    refreshRobotPositionControllers();
    refreshRobotCommand();
    onTerminate(s);
}

proto::RobotCommand Skill::rotateRobotCommand(proto::RobotCommand &cmd) {
    proto::RobotCommand output = cmd;
    output.mutable_vel()->set_x(- cmd.vel().x());
    output.mutable_vel()->set_y(- cmd.vel().y());
    output.set_w(static_cast<float>(control::ControlUtils::constrainAngle(cmd.w() + M_PI)));
    return output;
}

void Skill::refreshRobotCommand() {
    proto::RobotCommand emptyCmd;
    emptyCmd.set_use_angle(true);
    emptyCmd.set_id(robot ? robot->id : -1);
    emptyCmd.set_geneva_state(0);
    command = emptyCmd;
}

/// Velocity and acceleration limiters used on command
void Skill::limitRobotCommand() {

    bool isDefendPenaltyState = rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName== "keeper_penalty_defend_tactic";
    bool isKeeper = command.id() == robotDealer::RobotDealer::getKeeperID();

    auto limitedVel = Vector2(command.vel().x(), command.vel().y());
    limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
    if (!(isDefendPenaltyState&&isKeeper)){
        limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w());
    }
    robot->setPidPreviousVel(limitedVel);
    if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
        std::cout << "ERROR: ROBOT WILL HAVE NAN~!?!?!KLJ#Q@?LK@ " << node_name().c_str() << "!" << "  robot  " << robot->id << std::endl;
        robot->setPidPreviousVel(robot->vel);
    }
    command.mutable_vel()->set_x(limitedVel.x);
    command.mutable_vel()->set_y(limitedVel.y);
}


void Skill::refreshRobotPositionControllers() {
    robot->resetNumTreePosControl();
    robot->resetShotController();
    robot->resetBallHandlePosControl();
    robot->resetBasicPosControl();
}

} // rtt
