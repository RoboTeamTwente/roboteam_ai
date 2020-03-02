//
// Created by john on 3/2/20.
//

#include <roboteam_utils/Print.h>
#include "utilities/RobotDealer.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "control/ControlUtils.h"
#include "world_new/World.hpp"

#include "Skill.h"

namespace rtt::ai::stp {

    void Skill::rotateRobotCommand() noexcept {
        command.mutable_vel()->set_x(-command.vel().x());
        command.mutable_vel()->set_y(-command.vel().y());
        command.set_w(static_cast<float>(control::ControlUtils::constrainAngle(command.w() + M_PI)));
    }

    void Skill::publishRobotCommand() noexcept {
        if (!SETTINGS.isLeft()) {
            rotateRobotCommand();
        }

        limitRobotCommand();

        if (std::isnan(command.vel().x()) || std::isnan(command.vel().y())) {
            std::cout << "ERROR: x or y vel in command is NAN in Skill " << node_name().c_str() << "!"
                      << "  robot  " << robot->id << std::endl;
        }

        if (command.id() == -1) {
            if (robot && robot->getId() != -1) {
                command.set_id(robot->getId());
                io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
            }
        } else {
            io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
        }
        // refresh the robot command after it has been sent
        refreshRobotCommand();
    }

    void Skill::refreshRobotCommand() noexcept {
        proto::RobotCommand emptyCmd;
        emptyCmd.set_use_angle(true);
        emptyCmd.set_id(robot ? robot->getId() : -1);
        emptyCmd.set_geneva_state(0);
        command = emptyCmd;
    }

    void Skill::limitRobotCommand() noexcept {
        bool isDefendPenaltyState = rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName == "keeper_penalty_defend_tactic";
        bool isKeeper = command.id() == robotDealer::RobotDealer::getKeeperID();

        auto limitedVel = Vector2(command.vel().x(), command.vel().y());
        limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
        if (!(isDefendPenaltyState && isKeeper)) {
            limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w());
        }
        if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
            rtt_error("Robot will have NAN: " + name() + "!\nrobot: " + std::to_string(robot->getId()));
        }
        command.mutable_vel()->set_x(limitedVel.x);
        command.mutable_vel()->set_y(limitedVel.y);
    }

    void Skill::onTerminate() noexcept {
        terminate();
    }

    void Skill::onUpdate(SkillInfo const& info) noexcept {
        update(info);
    }

    void Skill::refreshRobotPositionControllers() const noexcept {
        rtt::world_new::World::instance()->getControllersForRobot(robot->getId())
                                                = world_new::robot::RobotControllers();
    }

    void Skill::onInitialize() noexcept {
        initialize();
    }

    constexpr const char* Skill::name() const noexcept {
        return "[abc] Skill";
    }
}
}