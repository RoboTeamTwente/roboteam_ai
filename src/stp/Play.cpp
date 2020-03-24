//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Play.hpp"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    calculateInfoForRoles();
    assignRoles();
}

void Play::updateWorld(world_new::World* world) noexcept {
    this->world = world;
    this->field = world->getField().value();
}

void Play::update() noexcept {
    // clear roleStatuses so it only contains the current tick's statuses
    roleStatuses.clear();

    if (world->getWorld()->getUs().size() != stpInfos.size()) {
        RTT_WARNING("Reassigning bots");

        // Make sure we don't re assign with too many robots
        if (world->getWorld()->getUs().size() > Constants::ROBOT_COUNT()) {
            RTT_ERROR("More robots than ROBOT_COUNT(), aborting update on Play")
            // Make sure the stpInfos is cleared to trigger a reassign whenever
            // the robots don't exceed ROBOT_COUNT anymore
            stpInfos = std::unordered_map<std::string, StpInfo>{};
            return;
        }
        assignRoles();
    }

    // Refresh the RobotViews, BallViews and fields
    refreshData();

    // derived class method call
    calculateInfoForRoles();

    for (auto& role : roles) {
        // Update the roles
        auto roleStatus = role->update(stpInfos[role->getName()]);
        roleStatuses.emplace_back(roleStatus);

        if (roleStatus == Status::Waiting) {
            // Should role skip end tactic?
            if (shouldRoleSkipEndTactic()) {
                // TODO: force role to go to next tactic
                // role.forceNextTactic(); (not implemented yet)
            }
        }
    }
}

bool Play::arePlayRolesFinished() {
    return std::all_of(roleStatuses.begin(), roleStatuses.end(), [](Status s) { return s == Status::Success; });
}

void Play::refreshData() noexcept {
    // Get a new BallView and field from world
    auto newBallView = world->getWorld()->getBall();
    auto newField = world->getField();

    for (auto& role : roles) {
        auto stpInfo = stpInfos.find(role->getName());
        if (stpInfo != stpInfos.end()) {
            // Get a new RobotView from world using the old robot id
            stpInfo->second.setRobot(world->getWorld()->getRobotForId(stpInfo->second.getRobot()->get()->getId()));

            // Assign the new BallView and field
            stpInfo->second.setBall(newBallView);
            stpInfo->second.setField(newField);
        }
    }
}
}  // namespace rtt::ai::stp
