//
// Created by jessevw on 24.03.20.
//

#include "stp/new_plays/Halt.h"
#include <stp/new_roles/TestRole.h>
#include "stp/invariants/WeHaveBallInvariant.h"

#include <utility>
#include "stp/new_roles/Halt.h"
namespace rtt::ai::stp::play {

    Halt::Halt(std::string playName) : Play(std::move(playName)) {
        invariants.clear();
        invariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Halt>(role::Halt("halt_0")),    std::make_unique<role::Halt>(role::Halt("halt_1")),
                std::make_unique<role::Halt>(role::Halt("halt_2")),    std::make_unique<role::Halt>(role::Halt("halt_3")),
                std::make_unique<role::Halt>(role::Halt("halt_4")),    std::make_unique<role::Halt>(role::Halt("halt_5")),
                std::make_unique<role::Halt>(role::Halt("halt_6")),    std::make_unique<role::Halt>(role::Halt("halt_7")),
                std::make_unique<role::Halt>(role::Halt("halt_8")),    std::make_unique<role::Halt>(role::Halt("halt_9")),
                std::make_unique<role::Halt>(role::Halt("halt_10"))};
    }

    uint8_t Halt::score(world_new::World* world) noexcept { return 14; }

    void Halt::calculateInfoForRoles() noexcept { }

    bool Halt::isValidPlayToStart(world_new::World* world) noexcept { return true; }

    bool Halt::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap Halt::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"halt_0", {closeToBallFlag}});
        flagMap.insert({"halt_1", {closeToTheirGoalFlag}});
        flagMap.insert({"halt_2", {notImportant}});
        flagMap.insert({"halt_3", {closeToTheirGoalFlag}});
        flagMap.insert({"halt_4", {closeToBallFlag}});
        flagMap.insert({"halt_5", {closeToTheirGoalFlag, closeToBallFlag}});
        flagMap.insert({"halt_6", {closeToBallFlag}});
        flagMap.insert({"halt_7", {closeToTheirGoalFlag}});
        flagMap.insert({"halt_8", {closeToTheirGoalFlag, closeToBallFlag}});
        flagMap.insert({"halt_9", {closeToBallFlag}});
        flagMap.insert({"halt_10", {closeToTheirGoalFlag}});
        return flagMap;
    }


}  // namespace rtt::ai::stp::play
