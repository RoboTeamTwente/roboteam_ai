//
// Created by jessevw on 24.03.20.
//

#include "stp/invariants/game_states/BallPlacementThemGameStateInvariant.h"
#include "stp/new_roles/BallPlacer.h"
#include "stp/new_roles/BallAvoider.h"
#include "stp/new_plays/BallPlacementThem.h"

namespace rtt::ai::stp::play {

    BallPlacementThem::BallPlacementThem() : Play() {
        startPlayInvariants.clear();
        startPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementThemGameStateInvariant>());

        keepPlayInvariants.clear();
        keepPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementThemGameStateInvariant>());

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::BallPlacer>(role::BallPlacer("ball_avoider_0")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_10"))};
    }

    uint8_t BallPlacementThem::score(world_new::World* world) noexcept { return 0; }

    void BallPlacementThem::calculateInfoForRoles() noexcept {
    }

    bool BallPlacementThem::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        flagMap.insert({"ball_avoider_0", {}});
        flagMap.insert({"ball_avoider_1", {}});
        flagMap.insert({"ball_avoider_2", {}});
        flagMap.insert({"ball_avoider_3", {}});
        flagMap.insert({"ball_avoider_4", {}});
        flagMap.insert({"ball_avoider_5", {}});
        flagMap.insert({"ball_avoider_6", {}});
        flagMap.insert({"ball_avoider_7", {}});
        flagMap.insert({"ball_avoider_8", {}});
        flagMap.insert({"ball_avoider_9", {}});
        flagMap.insert({"ball_avoider_10", {}});
        return flagMap;
    }

    const char *BallPlacementThem::getName() {
        return "Ball Placement Them";
    }
}  // namespace rtt::ai::stp::play
