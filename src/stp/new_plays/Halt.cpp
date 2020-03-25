//
// Created by jessevw on 24.03.20.
//

#include "include/roboteam_ai/stp/new_plays/Halt.h"
#include <stp/new_roles/TestRole.h>
#include "stp/new_roles/Halt.h"
namespace rtt::ai::stp::play {

    Halt::Halt() {
        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Halt>(role::Halt("halt0")), std::make_unique<role::Halt>(role::Halt("halt1")),
                std::make_unique<role::Halt>(role::Halt("halt2")),      std::make_unique<role::Halt>(role::Halt("halt3")),
                std::make_unique<role::Halt>(role::Halt("halt4")),    std::make_unique<role::Halt>(role::Halt("halt5")),
                std::make_unique<role::Halt>(role::Halt("halt6")),    std::make_unique<role::Halt>(role::Halt("halt7")),
                std::make_unique<role::Halt>(role::Halt("halt8")),    std::make_unique<role::Halt>(role::Halt("halt9")),
                std::make_unique<role::Halt>(role::Halt("halt10"))};
    }

    uint8_t Halt::score(world_new::World* world) noexcept { return 14; }

    void Halt::assignRoles() noexcept {
        Dealer dealer{world->getWorld().value(), &field};

        Dealer::FlagMap flagMap;
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"halt0", {closeToBallFlag}});
        flagMap.insert({"halt1", {closeToTheirGoalFlag}});
        flagMap.insert({"halt2", {notImportant}});
        flagMap.insert({"halt3", {closeToTheirGoalFlag}});
        flagMap.insert({"halt4", {closeToBallFlag}});
        flagMap.insert({"halt5", {closeToTheirGoalFlag, closeToBallFlag}});
        flagMap.insert({"halt6", {closeToBallFlag}});
        flagMap.insert({"halt7", {closeToTheirGoalFlag}});
        flagMap.insert({"halt8", {closeToTheirGoalFlag, closeToBallFlag}});
        flagMap.insert({"halt9", {closeToBallFlag}});
        flagMap.insert({"halt10", {closeToTheirGoalFlag}});

        auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap);

        stpInfos = std::unordered_map<std::string, StpInfo>{};
        for (auto& role : roles) {
            auto roleName{role->getName()};
            if (distribution.find(roleName) != distribution.end()) {
                auto robot = distribution.find(role->getName())->second;

                stpInfos.emplace(roleName, StpInfo{});
                stpInfos[roleName].setRobot(robot);
                stpInfos[roleName].setPositionToMoveTo(Vector2(robot->getId(), robot->getId()));
            }
        }
    }

    void Halt::calculateInfoForRoles() noexcept { }


    bool Halt::isValidPlayToStart(world_new::World* world) noexcept { return true; }

    bool Halt::isValidPlayToKeep(world_new::World* world) noexcept { return true; }

    bool Halt::shouldRoleSkipEndTactic() { return false; }

}  // namespace rtt::ai::stp::play
