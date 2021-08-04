//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/PenaltyUsPrepare.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

    PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Formation>(role::Formation("keeper")),
                std::make_unique<role::Formation>(role::Formation("kicker_formation")),
                std::make_unique<role::Formation>(role::Formation("formation_0")),
                std::make_unique<role::Formation>(role::Formation("formation_1")),
                std::make_unique<role::Formation>(role::Formation("formation_2")),
                std::make_unique<role::Formation>(role::Formation("formation_3")),
                std::make_unique<role::Formation>(role::Formation("formation_4")),
                std::make_unique<role::Formation>(role::Formation("formation_5")),
                std::make_unique<role::Formation>(role::Formation("formation_6")),
                std::make_unique<role::Formation>(role::Formation("formation_7")),
                std::make_unique<role::Formation>(role::Formation("formation_8"))};
    }

    uint8_t PenaltyUsPrepare::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::PenaltyUsPrepareGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
        const double xPosition = -4 * control_constants::ROBOT_RADIUS;
        const double distanceToCenterLine = field.getFieldWidth() / 2 - 2 * control_constants::ROBOT_RADIUS;
        const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine : -distanceToCenterLine;

        // Keeper
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

        // kicker, position right behind the ball
        stpInfos["kicker_formation"].setPositionToMoveTo(
                world->getWorld()->getBall()->get()->getPos() - Vector2{0.25, 0.0});

        // regular bots
        const std::string formation = "formation_";
        /// 1st row of 5 robots
        for (int i = 1; i <= 5; i++) {
            stpInfos[formation + std::to_string(i - 1)].setPositionToMoveTo(Vector2((i + 10) * xPosition, yPosition));
        }

        /// 2nd row of 4 robots
        for (int i = 6; i <= 9; i++) {
            stpInfos[formation + std::to_string(i - 1)].setPositionToMoveTo(Vector2((i + 5) * xPosition, yPosition + (2 * control_constants::ROBOT_RADIUS)));
        }
    }

    Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag kickerFormationFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"kicker_formation", {DealerFlagPriority::REQUIRED, {kickerFormationFlag}}});
        flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    const char *PenaltyUsPrepare::getName() { return "Penalty Us Prepare"; }
}  // namespace rtt::ai::stp::play
