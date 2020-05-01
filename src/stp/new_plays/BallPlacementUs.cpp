//
// Created by jessevw on 24.03.20.
//
#include "stp/Play.hpp"
#include "stp/invariants/BallPlacementUsGameStateInvariant.h"
#include "stp/new_roles/BallPlacer.h"
#include "stp/new_roles/BallAvoider.h"
#include "stp/new_plays/BallPlacementUs.h"
namespace rtt::ai::stp::play {

    BallPlacementUs::BallPlacementUs() : Play() {
        // TODO: decide start invariants
        startPlayInvariants.clear();
        startPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementUsGameStateInvariant>());

        // TODO: decide keep invariants
        keepPlayInvariants.clear();
        keepPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementUsGameStateInvariant>());

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),    std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_10"))};
    }

    uint8_t BallPlacementUs::score(world_new::World* world) noexcept { return 100; }

    void BallPlacementUs::calculateInfoForRoles() noexcept {
        if (stpInfos.find("ball_placer") != stpInfos.end()) {
            auto ballTarget = Vector2(0,0);
            stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
        }
        else {
            RTT_ERROR("No ball placement robot assigned!")
        }
    }

    bool BallPlacementUs::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"ball_placer", {closeToBallFlag}});
        flagMap.insert({"ball_avoider_1", {notImportant}});
        flagMap.insert({"ball_avoider_2", {notImportant}});
        flagMap.insert({"ball_avoider_3", {notImportant}});
        flagMap.insert({"ball_avoider_4", {notImportant}});
        flagMap.insert({"ball_avoider_5", {notImportant}});
        flagMap.insert({"ball_avoider_6", {notImportant}});
        flagMap.insert({"ball_avoider_7", {notImportant}});
        flagMap.insert({"ball_avoider_8", {notImportant}});
        flagMap.insert({"ball_avoider_9", {notImportant}});
        flagMap.insert({"ball_avoider_10", {notImportant}});
        return flagMap;
    }

    const char *BallPlacementUs::getName() {
        return "Ball Placement Us";
    }
}  // namespace rtt::ai::stp::play
