//
// Created by john on 3/9/20.
//

#include "stp/Play.hpp"

#include "interface/widgets/MainControlsWidget.h"

namespace rtt::ai::stp {

    void Play::initialize(gen::PlayInfos &_previousPlayInfos) noexcept {
//        previousPlayInfos = _previousPlayInfos;
//        if (!previousPlayInfos->empty()) {
//            RTT_DEBUG(
//                std::to_string(previousPlayInfos->begin()->second.robotID.value_or(-1)));
//        }
        calculateInfoForRoles();
        distributeRoles();
        previousRobotNum = world->getWorld()->getRobotsNonOwning().size();
    }

    void Play::updateWorld(world::World *world) noexcept {
        this->world = world;
        this->field = world->getField().value();
    }

    void Play::update() noexcept {
        // clear roleStatuses so it only contains the current tick's statuses
        roleStatuses.clear();
        RTT_INFO("Play executing: ", getName())

        // Check if the amount of robots changed
        // If so, we will re deal the roles
        auto currentRobotNum{world->getWorld()->getRobotsNonOwning().size()};

        if (currentRobotNum != previousRobotNum) {
            RTT_INFO("Reassigning bots")
            reassignRobots();
            previousRobotNum = currentRobotNum;
        }

        // Refresh the RobotViews, BallViews and fields
        refreshData();

        // derived class method call
        calculateInfoForRoles();

        // Loop through roles and update them if they exist in stpInfos
        for (auto &role : roles) {
            if (stpInfos.find(role->getName()) != stpInfos.end()) {
                // Update and store the returned status
                auto roleStatus = role->update(stpInfos[role->getName()]);
                roleStatuses[role.get()] = roleStatus;
            } else {
                RTT_DEBUG("Trying to update role [", role->getName(), "] which is not in STPInfos!")
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
        calculateInfoForRoles();
        distributeRoles();
    }

    void Play::refreshData() noexcept {
        // Get a new BallView and field from world
        auto newBallView = world->getWorld()->getBall();

        // Loop through all roles, if an stpInfo exists and has an assigned robot, refresh the data
        for (auto &role : roles) {
            auto stpInfo = stpInfos.find(role->getName());
            if (stpInfo != stpInfos.end() && stpInfo->second.getRobot().has_value()) {
                // Get a new RobotView from world using the old robot id
                stpInfo->second.setRobot(world->getWorld()->getRobotForId(stpInfo->second.getRobot()->get()->getId()));

                // Assign the new BallView and field
                stpInfo->second.setBall(newBallView);
                stpInfo->second.setField(field);

                if (stpInfo->second.getEnemyRobot().has_value()) {
                    stpInfo->second.setEnemyRobot(
                            world->getWorld()->getRobotForId(stpInfo->second.getEnemyRobot()->get()->getId(), false));
                }
            }
        }
    }

    void Play::distributeRoles() noexcept {
        Dealer dealer{world->getWorld().value(), &field};
        auto flagMap = decideRoleFlags();
        auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap, stpInfos);

        // TODO-Max if role exists in oldStpInfos then copy those.
        // Clear the stpInfos for the new role assignment
        for (auto &role : roles) {
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

    bool Play::isValidPlayToKeep(PlayEvaluator &playEvaluator) noexcept {
        return (interface::MainControlsWidget::ignoreInvariants || shouldEndPlay() ||
                std::all_of(keepPlayEvaluation.begin(), keepPlayEvaluation.end(), [&playEvaluator](auto &x) {
                    return playEvaluator.checkEvaluation(x);
                }));
    }

    bool Play::isValidPlayToStart(PlayEvaluator &playEvaluator) const noexcept {
        return (interface::MainControlsWidget::ignoreInvariants ||
                std::all_of(startPlayEvaluation.begin(), startPlayEvaluation.end(), [&playEvaluator](auto &x) {
                    return playEvaluator.checkEvaluation(x);
                }));
    }

    void Play::calculateInfoForScoredRoles(world::World* _world) noexcept {}

    uint8_t Play::getLastScore() const {
        return lastScore.value_or(0);
    }

    void Play::storePlayInfo(gen::PlayInfos &previousPlayInfo) noexcept {}

    bool Play::shouldEndPlay() noexcept { return false; }

    void Play::initRoles() noexcept {
        stpInfos = std::unordered_map<std::string, StpInfo>{};
        for (auto &role : roles) {
            role->reset();
            auto roleName{role->getName()};
            stpInfos.emplace(roleName, StpInfo{});
        }
    }
}  // namespace rtt::ai::stp
