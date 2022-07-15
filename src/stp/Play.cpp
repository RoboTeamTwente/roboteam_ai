//
// Created by john on 3/9/20.
//

#include "stp/Play.hpp"

#include "control/ControlUtils.h"
#include "interface/widgets/MainControlsWidget.h"

namespace rtt::ai::stp {

void Play::initialize(gen::PlayInfos &_previousPlayInfos) noexcept {
    //        previousPlayInfos = _previousPlayInfos;
    //        if (!previousPlayInfos->empty()) {
    //            RTT_DEBUG(
    //                std::to_string(previousPlayInfos->begin()->second.robotID.value_or(-1)));
    //        }
    stpInfos.clear();
    for (auto &role : roles) {
        if (role != nullptr) role->reset();
    }
    calculateInfoForRoles();
    distributeRoles();
    previousRobotNum = world->getWorld()->getRobotsNonOwning().size();
    previousKeeperId = GameStateManager::getCurrentGameState().keeperId;
}

void Play::setWorld(world::World *world) noexcept { this->world = world; }

void Play::updateField(world::Field field) noexcept { this->field = field; }

void Play::update() noexcept {
    // clear roleStatuses so it only contains the current tick's statuses
    roleStatuses.clear();
    //    RTT_INFO("Play executing: ", getName())

    // Check if the amount of robots changed or keeper id changed
    // If so, we will re deal the roles
    auto currentRobotNum{world->getWorld()->getRobotsNonOwning().size()};
    auto currentKeeperId = GameStateManager::getCurrentGameState().keeperId;

    if (currentRobotNum != previousRobotNum || currentKeeperId != previousKeeperId) {
        //        RTT_INFO("Reassigning bots")
        reassignRobots();
        previousRobotNum = currentRobotNum;
        previousKeeperId = currentKeeperId;
    }

    // Refresh the RobotViews, BallViews and fields
    refreshData();

    // derived class method call
    calculateInfoForRoles();

    // Loop through roles and update them if they are assigned to a robot
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto stpInfo = stpInfos.find(role->getName());
        if (stpInfo != stpInfos.end() && stpInfo->second.getRobot()) {
            // Update and store the returned status
            auto roleStatus = role->update(stpInfos[role->getName()]);
            roleStatuses[role.get()] = roleStatus;
        }
    }
}

void Play::reassignRobots() noexcept {
    // Make sure we don't reassign when there are more robots than MAX_ROBOT_COUNT
    if (world->getWorld()->getUs().size() > stp::control_constants::MAX_ROBOT_COUNT) {
        RTT_ERROR("More robots than ROBOT_COUNT(), aborting update on Play")
        // Make sure the stpInfos is cleared to trigger a reassign whenever
        // the robots don't exceed ROBOT_COUNT anymore
        stpInfos.clear();
        return;
    }
    stpInfos.clear();
    calculateInfoForRoles();
    distributeRoles();
}

void Play::refreshData() noexcept {
    // Get a new BallView and field from world
    auto newBallView = world->getWorld()->getBall();

    // Loop through all roles, if an stpInfo exists and has an assigned robot, refresh the data
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto stpInfo = stpInfos.find(role->getName());
        if (stpInfo != stpInfos.end() && stpInfo->second.getRobot().has_value()) {
            // Get a new RobotView from world using the old robot id
            stpInfo->second.setRobot(world->getWorld()->getRobotForId(stpInfo->second.getRobot()->get()->getId()));

            // Set max velocity depending on the gamestate rules and whether we have the ball
            if (stpInfo->second.getRobot()) stpInfo->second.setMaxRobotVelocity(control::ControlUtils::getMaxVelocity(stpInfo->second.getRobot().value()->hasBall()));

            // The keeper does not need to avoid our defense area
            if (stpInfo->second.getRoleName() == "keeper") stpInfo->second.setShouldAvoidDefenseArea(false);

            // Assign the new BallView and field
            stpInfo->second.setBall(newBallView);
            stpInfo->second.setField(field);

            if (stpInfo->second.getEnemyRobot().has_value()) {
                stpInfo->second.setEnemyRobot(world->getWorld()->getRobotForId(stpInfo->second.getEnemyRobot()->get()->getId(), false));
            }
        }
    }
}

void Play::distributeRoles() noexcept {
    Dealer dealer{world->getWorld().value(), &field};

    // Set role names for each stpInfo
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto roleName{role->getName()};
        stpInfos[roleName].setRoleName(roleName);
    }

    auto flagMap = decideRoleFlags();
    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap, stpInfos);

    // TODO-Max if role exists in oldStpInfos then copy those.
    // Clear the stpInfos for the new role assignment
    for (auto &role : roles) {
        if (role == nullptr) continue;
        role->reset();
        auto roleName{role->getName()};
        if (distribution.find(roleName) != distribution.end()) {
            auto robot = distribution.find(role->getName())->second;
            stpInfos[roleName].setRobot(robot);
        }
    }
    std::for_each(stpInfos.begin(), stpInfos.end(), [this](auto &each) { each.second.setCurrentWorld(world); });
}

std::unordered_map<Role *, Status> const &Play::getRoleStatuses() const { return roleStatuses; }

bool Play::isValidPlayToKeep() noexcept {
    return (interface::MainControlsWidget::ignoreInvariants ||
            (!shouldEndPlay() && std::all_of(keepPlayEvaluation.begin(), keepPlayEvaluation.end(), [this](auto &x) { return PlayEvaluator::checkEvaluation(x, world); })));
}

bool Play::isValidPlayToStart() const noexcept {
    return (interface::MainControlsWidget::ignoreInvariants ||
            std::all_of(startPlayEvaluation.begin(), startPlayEvaluation.end(), [this](auto &x) { return PlayEvaluator::checkEvaluation(x, world); }));
}

void Play::calculateInfoForScoredRoles(world::World *_world) noexcept {}

uint8_t Play::getLastScore() const { return lastScore.value_or(0); }

void Play::storePlayInfo(gen::PlayInfos &previousPlayInfo) noexcept {}

bool Play::shouldEndPlay() noexcept { return false; }

}  // namespace rtt::ai::stp
