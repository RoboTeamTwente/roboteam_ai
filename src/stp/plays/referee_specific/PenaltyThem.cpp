//
// Created by timovdk on 4/28/20.
//

#include "stp/plays/referee_specific/PenaltyThem.h"
#include "stp/roles/passive/Halt.h"
#include "stp/roles/PenaltyKeeper.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

    PenaltyThem::PenaltyThem() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyThemGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyThemGameState);

        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
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

    uint8_t PenaltyThem::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::PenaltyThemGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    Dealer::FlagMap PenaltyThem::decideRoleFlags() const noexcept {
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

    void PenaltyThem::calculateInfoForRoles() noexcept {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter().x,world->getWorld()->getBall()->get()->getPos().y));
        stpInfos["keeper"].setPositionToShootAt(
                world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    }

    const char *PenaltyThem::getName() { return "Penalty Them"; }
}  // namespace rtt::ai::stp::play