//
// Created by floris on 08-06-21.
//

#include "stp/plays/HardwareChallenge1.h"

#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"
#include "stp/roles/HardwareChallenge1.h"
#include "stp/roles/passive/Halt.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

    HardwareChallenge1::HardwareChallenge1() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::HardwareChallenge1>("HardwareChallenge1_0"),
            std::make_unique<role::Halt>("halt_0"),
            std::make_unique<role::Halt>("halt_1"),
            std::make_unique<role::Halt>("halt_2"),
            std::make_unique<role::Halt>("halt_3"),
            std::make_unique<role::Halt>("halt_4"),
            std::make_unique<role::Halt>("halt_5"),
            std::make_unique<role::Halt>("halt_6"),
            std::make_unique<role::Halt>("halt_7"),
            std::make_unique<role::Halt>("halt_8"),
            std::make_unique<role::Halt>("halt_9")};
    }

    uint8_t HardwareChallenge1::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::HaltGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    Dealer::FlagMap HardwareChallenge1::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag attackerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);

        flagMap.insert({"HardwareChallenge1_0", {DealerFlagPriority::REQUIRED, {attackerFlag}}});
        const std::string roleName = "halt_";
        for (int i = 1; i <= 9; i++) {
            flagMap.insert({roleName + std::to_string(i), {DealerFlagPriority::LOW_PRIORITY, {}}});
        }

        return flagMap;
    }

    void HardwareChallenge1::calculateInfoForRoles() noexcept {
        auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
        stpInfos["HardwareChallenge1_0"].setPositionToShootAt(goalTarget);
        stpInfos["HardwareChallenge1_0"].setPositionToMoveTo(pos::getPosition(stpInfos["HardwareChallenge1_0"].getPositionToMoveTo(),gen::gridRightMid, gen::GoalShootPosition, field, world));
        stpInfos["HardwareChallenge1_0"].setShotType(ShotType::MAX);
        //TODO: Calculate a location to drive to with the ball
        //TODO: If we know a location to drive to, make sure the robot positions itself on the best side with GetBehindBallInDirection
    }

    void HardwareChallenge1::calculateInfoForScoredRoles(world::World *) noexcept {}

    const char *HardwareChallenge1::getName() { return "HardwareChallenge1"; }
}  // namespace rtt::ai::stp::play
