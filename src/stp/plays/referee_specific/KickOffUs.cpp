//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/KickOffUs.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Halt.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

KickOffUs::KickOffUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::KickOffUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::KickOffUsGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>("keeper"), std::make_unique<role::Passer>("passer"), std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Halt>("halt_0"),   std::make_unique<role::Halt>("halt_1"),   std::make_unique<role::Halt>("halt_2"),
        std::make_unique<role::Halt>("halt_3"),   std::make_unique<role::Halt>("halt_4"),   std::make_unique<role::Halt>("halt_5"),
        std::make_unique<role::Halt>("halt_6"),   std::make_unique<role::Halt>("halt_7")};
}

uint8_t KickOffUs::score(PlayEvaluator &playEvaluator) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::KickOffUsGameState), 1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();  // DONT TOUCH.
}

void KickOffUs::calculateInfoForRoles() noexcept {
    // Keeper
    // TODO: set good position to shoot at (compute pass location)- possibly do this in the keeper role
    stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // Kicker
    // TODO: set good position to shoot at (compute pass location)- possibly do this in the passer role
    stpInfos["passer"].setPositionToShootAt(field.getTheirGoalCenter());
    stpInfos["passer"].setShotType(ShotType::PASS);
    stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    // TODO: set good position to move to after pass
    stpInfos["passer"].setPositionToMoveTo(Vector2(0, 0));

    // Receiver
    // TODO: set receiving position based on pass computation
    stpInfos["receiver"].setPositionToMoveTo(Vector2{-1.0, 1.0});
}

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {kickerFlag}}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

bool KickOffUs::shouldEndPlay() noexcept { return false; }

const char *KickOffUs::getName() { return "Kick Off Us"; }
}  // namespace rtt::ai::stp::play
