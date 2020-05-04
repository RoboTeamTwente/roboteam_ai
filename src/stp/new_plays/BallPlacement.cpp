//
// Created by jessevw on 24.03.20.
//

#include "stp/new_plays/Halt.h"
#include "stp/invariants/game_states/HaltGameStateInvariant.h"
#include "stp/invariants/WeHaveBallInvariant.h"
#include "stp/new_roles/BallPlacer.h"
#include "stp/new_roles/Halt.h"
#include "stp/new_plays/BallPlacement.h"
namespace rtt::ai::stp::play {

    BallPlacement::BallPlacement() : Play() {
        // TODO: decide start invariants
        startPlayInvariants.clear();
        startPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());

        // TODO: decide keep invariants
        keepPlayInvariants.clear();
        keepPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")),    std::make_unique<role::Halt>(role::Halt("halt_1")),
                std::make_unique<role::Halt>(role::Halt("halt_2")),    std::make_unique<role::Halt>(role::Halt("halt_3")),
                std::make_unique<role::Halt>(role::Halt("halt_4")),    std::make_unique<role::Halt>(role::Halt("halt_5")),
                std::make_unique<role::Halt>(role::Halt("halt_6")),    std::make_unique<role::Halt>(role::Halt("halt_7")),
                std::make_unique<role::Halt>(role::Halt("halt_8")),    std::make_unique<role::Halt>(role::Halt("halt_9")),
                std::make_unique<role::Halt>(role::Halt("halt_10"))};
    }

    uint8_t BallPlacement::score(world_new::World* world) noexcept { return 0; }

    void BallPlacement::calculateInfoForRoles() noexcept {
        if (stpInfos.find("ball_placer") != stpInfos.end()) {
            auto ballTarget = Vector2(0,0);
            stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
        }
        else {
            RTT_ERROR("No ball placement robot assigned!")
        }
    }

    bool BallPlacement::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap BallPlacement::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"ball_placer", {closeToBallFlag}});
        flagMap.insert({"halt_1", {notImportant}});
        flagMap.insert({"halt_2", {notImportant}});
        flagMap.insert({"halt_3", {notImportant}});
        flagMap.insert({"halt_4", {notImportant}});
        flagMap.insert({"halt_5", {notImportant}});
        flagMap.insert({"halt_6", {notImportant}});
        flagMap.insert({"halt_7", {notImportant}});
        flagMap.insert({"halt_8", {notImportant}});
        flagMap.insert({"halt_9", {notImportant}});
        flagMap.insert({"halt_10", {notImportant}});
        return flagMap;
    }

    const char *BallPlacement::getName() {
        return "Ball Placement";
    }
}  // namespace rtt::ai::stp::play
