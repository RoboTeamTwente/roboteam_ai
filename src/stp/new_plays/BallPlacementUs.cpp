//
// Created by jessevw on 24.03.20.
//
#include "stp/invariants/game_states/BallPlacementUsGameStateInvariant.h"
#include "stp/new_roles/BallPlacer.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/BallAvoider.h"
#include "stp/new_plays/BallPlacementUs.h"
#include "utilities/GameStateManager.hpp"

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
                std::make_unique<role::Keeper>(role::Keeper("keeper"))};
    }

    uint8_t BallPlacementUs::score(world_new::World* world) noexcept { return 100; }

    void BallPlacementUs::calculateInfoForRoles() noexcept {
        if (stpInfos.find("ball_placer") != stpInfos.end()) {
            auto ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
            stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
        }
        if (stpInfos.find("keeper") != stpInfos.end()) {
            stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::Team::them));
        }
        else {
            RTT_ERROR("No ball placement robot assigned!")
        }
    }

    bool BallPlacementUs::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag ball_placement(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
        Dealer::DealerFlag keeper(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
        Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);
        flagMap.insert({"ball_placer", {ball_placement}});
        flagMap.insert({"ball_avoider_1", {not_important}});
        flagMap.insert({"ball_avoider_2", {not_important}});
        flagMap.insert({"ball_avoider_3", {not_important}});
        flagMap.insert({"ball_avoider_4", {not_important}});
        flagMap.insert({"ball_avoider_5", {not_important}});
        flagMap.insert({"ball_avoider_6", {not_important}});
        flagMap.insert({"ball_avoider_7", {not_important}});
        flagMap.insert({"ball_avoider_8", {not_important}});
        flagMap.insert({"ball_avoider_9", {not_important}});
        flagMap.insert({"keeper", {keeper}});
        return flagMap;
    }

    const char *BallPlacementUs::getName() {
        return "Ball Placement Us";
    }
}  // namespace rtt::ai::stp::play
