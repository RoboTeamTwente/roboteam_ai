//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/KickOffThem.h"
#include "stp/roles/passive/Halt.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

    KickOffThem::KickOffThem() : Play() {
        startPlayEvaluation.clear(); // DONT TOUCH.
        startPlayEvaluation.emplace_back(eval::KickOffThemGameState);

        keepPlayEvaluation.clear(); // DONT TOUCH.
        keepPlayEvaluation.emplace_back(eval::KickOffThemGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::Halt>(role::Halt("halt_0")),
                std::make_unique<role::Halt>(role::Halt("halt_1")),
                std::make_unique<role::Halt>(role::Halt("halt_2")),
                std::make_unique<role::Halt>(role::Halt("halt_3")),
                std::make_unique<role::Halt>(role::Halt("halt_4")),
                std::make_unique<role::Halt>(role::Halt("halt_5")),
                std::make_unique<role::Halt>(role::Halt("halt_6")),
                std::make_unique<role::Halt>(role::Halt("halt_7")),
                std::make_unique<role::Halt>(role::Halt("halt_8")),
                std::make_unique<role::Halt>(role::Halt("halt_9"))};
    }

    uint8_t KickOffThem::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::KickOffThemGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void KickOffThem::calculateInfoForRoles() noexcept {
        // Keeper
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
        stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    }

    Dealer::FlagMap KickOffThem::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    const char *KickOffThem::getName() { return "Kick Off Them"; }

}  // namespace rtt::ai::stp::play
