//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Play.hpp"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    assignRoles();
}

void Play::updateWorld(world_new::World* world) noexcept {
    this->world = world;
    this->field = world->getField().value();
}

Status Play::update() noexcept {
    if(world->getWorld()->getUs().size() != stpInfos.size()) {
        RTT_WARNING("Reassigning bots");
        assignRoles();
    }

    // Connect roles to robotIDs, and set some other basic info each play needs
    for (auto& role : roles) {
        auto roleName{role->getName()};
        if(stpInfos.find(roleName) != stpInfos.end()) {
            // TODO refresh robots
            stpInfos[roleName].setRobot(world->getWorld()->getRobotForId(stpInfos.find(roleName)->second.getRobot()->get()->getId()));
            stpInfos[roleName].setBall(world->getWorld()->getBall());
            stpInfos[roleName].setField(world->getField());
        }
    }

    // derived class method call
    calculateInfoForPlay();

    for (auto& role : roles) {
        // Update the roles
        auto roleStatus = role->update(stpInfos[role->getName()]);
        roleStatuses.emplace_back(roleStatus);
        if (roleStatus == Status::Success) {
            // TODO: Update default tactic here
        }
        if (roleStatus == Status::Waiting) {
            // Should role skip end tactic?
            if(shouldRoleSkipEndTactic()) {
                // TODO: force role to go to next tactic
                // role.forceNextTactic(); (not implemented yet)
            }
        }
    }
    // Play is done when all statuses of roles are success. Keep in mind that a role that is in an endtactic will also
    // return succes even if its tactic is not done
    arePlayRolesFinished();
}

    bool Play::shouldRoleSkipEndTactic() {
        return false;
    }

    bool Play::arePlayRolesFinished() {
        return std::all_of(roleStatuses.begin(), roleStatuses.end(), [](Status s){return s == Status::Success;});
    }
}  // namespace rtt::ai::stp
