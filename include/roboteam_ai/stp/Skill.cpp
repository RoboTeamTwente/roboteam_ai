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

    proto::RobotCommand Skill::rotateRobotCommand(proto::RobotCommand const& cmd) const noexcept {
        proto::RobotCommand output = cmd;
        output.mutable_vel()->set_x(-cmd.vel().x());
        output.mutable_vel()->set_y(-cmd.vel().y());
        output.set_w(static_cast<float>(control::ControlUtils::constrainAngle(cmd.w() + M_PI)));
        return output;
    }

    void Skill::publishRobotCommand() noexcept {
        if (!SETTINGS.isLeft()) {
            command = rotateRobotCommand(command);
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

    void Skill::refreshRobotCommand() noexcept {}

    void Skill::limitRobotCommand() noexcept {
        bool isDefendPenaltyState = rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName == "keeper_penalty_defend_tactic";
        bool isKeeper = command.id() == robotDealer::RobotDealer::getKeeperID();

        auto limitedVel = Vector2(command.vel().x(), command.vel().y());
        limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
        if (!(isDefendPenaltyState && isKeeper)) {
            limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w());
        }
        if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
            rtt_error("Robot will have NAN: " + node_name() + "!\nrobot: " + std::to_string(robot->getId()));
        }
        command.mutable_vel()->set_x(limitedVel.x);
        command.mutable_vel()->set_y(limitedVel.y);
    }

    void Skill::onTerminate(Status s) noexcept {
        terminate();
    }

    void Skill::onUpdate() noexcept {
        update();
    }

    void Skill::refreshRobotPositionControllers() const noexcept {
        rtt::world_new::World::instance()->getControllersForRobot(robot->getId())
                                                = world_new::robot::RobotControllers();
    }

    void Skill::onInitialize() noexcept {
        initialize();
    }
}
}