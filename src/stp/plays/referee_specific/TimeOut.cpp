//
// Created by jordi on 01-05-20.
//

#include "stp/plays/referee_specific/TimeOut.h"

#include "stp/evaluations/game_states/TimeOutGameStateEvaluation.h"
#include "stp/roles/passive/BallAvoider.h"
#include  "stp/roles/PenaltyKeeper.h"

namespace rtt::ai::stp::play {

    TimeOut::TimeOut() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::TimeOutGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::TimeOutGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::PenaltyKeeper>("PenaltyKeeper"),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_1")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_2")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_3")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_4")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_5")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_6")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_7")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_8")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_9")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("time_out_10"))};
    }

    uint8_t TimeOut::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::TimeOutGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void TimeOut::calculateInfoForRoles() noexcept {
        const auto xPosition = 4 * control_constants::ROBOT_RADIUS;
        const double distanceToCenterLine = field.getFieldWidth() / 2 - 2 * control_constants::ROBOT_RADIUS;
        const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine : -distanceToCenterLine;

        const std::string roleName = "time_out_";
        stpInfos["PenaltyKeeper"].setPositionToMoveTo(field.getOurGoalCenter());

        /// 1st row of 5 robots
        for (int i = 1; i <= 5; i++) {
            stpInfos[roleName + std::to_string(i)].setPositionToMoveTo(Vector2((i + 10) * xPosition, yPosition));
        }

        /// 2nd row of 5 robots
        for (int i = 6; i <= 10; i++) {
            stpInfos[roleName + std::to_string(i)].setPositionToMoveTo(Vector2((i + 5) * xPosition, yPosition + (2 * control_constants::ROBOT_RADIUS)));
        }
    }

    Dealer::FlagMap TimeOut::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        const std::string roleName = "time_out_";
        flagMap.insert({"PenaltyKeeper", {DealerFlagPriority::KEEPER, {}}});
        for (int i = 1; i <= 10; i++) {
            flagMap.insert({roleName + std::to_string(i), {DealerFlagPriority::LOW_PRIORITY, {}}});
        }

        return flagMap;
    }

    const char *TimeOut::getName() { return "Time Out"; }

}  // namespace rtt::ai::stp::play