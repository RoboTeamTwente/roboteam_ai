//
// Created by jessevw on 21.02.20.
//

#include "include/roboteam_ai/new_skills/NewSkill.h"
#include "skills/Skill.h"

#include <include/roboteam_ai/utilities/Settings.h>

#include <cmath>
#include <include/roboteam_ai/world_new/views/RobotView.hpp>

#include "control/ControlUtils.h"
#include "utilities/Constants.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/RobotDealer.h"
#include "world/Ball.h"
#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai {

    NewSkill::NewSkill(std::string name, bt::Blackboard::Ptr blackboard) : bt::Leaf(std::move(name), std::move(blackboard)) {
        robot = std::make_shared<Robot>();
        ball = std::make_shared<Ball>();
    }

    void NewSkill::publishRobotCommand() {
        if (!SETTINGS.isLeft()) {
            command = rotateRobotCommand(command);
        }

        limitRobotCommand();

        if (command.id() == -1) {
            std::cerr << "[NewSkill::publishRobotCommand] Trying to send a RobotCommand to id " << command.id() << std::endl;
            if (robot && robot->id != -1) {
                command.set_id(robot->id);
                io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
            }
        } else {
            io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
        }
        // refresh the robotcommand after it has been sent
        refreshRobotCommand();
    }

    std::string NewSkill::node_name() { return name; }

    NewSkill::Status NewSkill::update() {
        robotId = -15;// TODO: after the tactic is implemented, needs to be some way the skill figures out which id to send to. Should be known by the tactic
        updateRobot();
        ball = world->getBall();  // update ball position
        if (!robot){
            std::cerr << "[NewSkill::update] The robot is not initialized" << std::endl;
            return Status::Failure;
        }
        if (robot->id == -1) {
            std::cerr << "[NewSkill::update] The robot id is" << robot->id << std::endl;
            return Status::Failure;
        }
        if (!ball) {
            return Status::Waiting;
        }
        return onUpdate();
    }

    void NewSkill::initialize() {
        // TODO: be able to get robot from dealer
        // robot = dealer.distributions.at(role).robot;
        ball = world->getBall();
        if (!robot || robot->id == -1) return;
        if (!ball) return;
        refreshRobotCommand();
        onInitialize();
    }

    void NewSkill::terminate(Status s) {
        if (!init) {
            return;
        }
        init = false;
        if (!robot || robot->id == -1) return;
        if (!ball) return;
        refreshRobotPositionControllers();
        refreshRobotCommand();
        onTerminate(s);
    }

    proto::RobotCommand NewSkill::rotateRobotCommand(proto::RobotCommand &cmd) {
        proto::RobotCommand output = cmd;
        output.mutable_vel()->set_x(-cmd.vel().x());
        output.mutable_vel()->set_y(-cmd.vel().y());
        output.set_w(static_cast<float>(control::ControlUtils::constrainAngle(cmd.w() + M_PI)));
        return output;
    }

    void NewSkill::refreshRobotCommand() {
        proto::RobotCommand emptyCmd;
        emptyCmd.set_use_angle(true);
        emptyCmd.set_id(robot ? robot->id : -1);
        emptyCmd.set_geneva_state(0);
        command = emptyCmd;
    }

/// Velocity and acceleration limiters used on command. TODO: this function is nasty, need to actually make it do what it says it does
    void NewSkill::limitRobotCommand() {
        bool isDefendPenaltyState = rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName == "keeper_penalty_defend_tactic";
        bool isKeeper = command.id() == robotDealer::RobotDealer::getKeeperID();

        auto limitedVel = Vector2(command.vel().x(), command.vel().y());
        limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
        if (!(isDefendPenaltyState && isKeeper)) {
            limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w());
        }
        robot->setPidPreviousVel(limitedVel);
        if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
            std::cerr << "[NewSkill::publishRobotCommand] The robot has a velocity of NaN: " << node_name().c_str() << "!"
                      << "  robot  " << robot->id << std::endl;
            robot->setPidPreviousVel(robot->vel);
        }
        command.mutable_vel()->set_x(limitedVel.x);
        command.mutable_vel()->set_y(limitedVel.y);
    }

    void NewSkill::refreshRobotPositionControllers() {
        robot->resetNumTreePosControl();
        robot->resetShotController();
        robot->resetBallHandlePosControl();
        robot->resetBasicPosControl();
    }

    void NewSkill::onInitialize() {
    }

}  // namespace rtt::ai
