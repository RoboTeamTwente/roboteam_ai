//
// Created by timovdk on 5/1/20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyThemPrepare.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

    PenaltyThemPrepare::PenaltyThemPrepare() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Formation>(role::Formation("keeper")),
                std::make_unique<role::Formation>(role::Formation("formation_0")),
                std::make_unique<role::Formation>(role::Formation("formation_1")),
                std::make_unique<role::Formation>(role::Formation("formation_2")),
                std::make_unique<role::Formation>(role::Formation("formation_3")),
                std::make_unique<role::Formation>(role::Formation("formation_4")),
                std::make_unique<role::Formation>(role::Formation("formation_5")),
                std::make_unique<role::Formation>(role::Formation("formation_6")),
                std::make_unique<role::Formation>(role::Formation("formation_7")),
                std::make_unique<role::Formation>(role::Formation("formation_8")),
                std::make_unique<role::Formation>(role::Formation("formation_9"))};
    }

    uint8_t PenaltyThemPrepare::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::PenaltyThemPrepareGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void PenaltyThemPrepare::calculateInfoForRoles() noexcept {
        const double xPosition = -4 * control_constants::ROBOT_RADIUS;
        const double distanceToCenterLine = field.getFieldWidth() / 2 - 2 * control_constants::ROBOT_RADIUS;
        const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine : -distanceToCenterLine;

        // Keeper
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

        // regular bots
        const std::string formation = "formation_";
        for (int i = 1; i <= 10; i++) {
            stpInfos[formation + std::to_string(i - 1)].setPositionToMoveTo(Vector2(i * xPosition, yPosition));
        }
    }

    Dealer::FlagMap PenaltyThemPrepare::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_9", {DealerFlagPriority::LOW_PRIORITY, {}}});
        return flagMap;
    }

    const char *PenaltyThemPrepare::getName() { return "Penalty Them Prepare"; }

}  // namespace rtt::ai::stp::play
