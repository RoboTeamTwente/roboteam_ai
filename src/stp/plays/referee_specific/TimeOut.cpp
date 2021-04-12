//
// Created by jordi on 01-05-20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/TimeOut.h"

#include "stp/evaluations/game_states/TimeOutGameStateEvaluation.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

    TimeOut::TimeOut() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::TimeOutGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::TimeOutGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Formation>(role::Formation("time_out_1")),
                std::make_unique<role::Formation>(role::Formation("time_out_2")),
                std::make_unique<role::Formation>(role::Formation("time_out_3")),
                std::make_unique<role::Formation>(role::Formation("time_out_4")),
                std::make_unique<role::Formation>(role::Formation("time_out_5")),
                std::make_unique<role::Formation>(role::Formation("time_out_6")),
                std::make_unique<role::Formation>(role::Formation("time_out_7")),
                std::make_unique<role::Formation>(role::Formation("time_out_8")),
                std::make_unique<role::Formation>(role::Formation("time_out_9")),
                std::make_unique<role::Formation>(role::Formation("time_out_10")),
                std::make_unique<role::Formation>(role::Formation("time_out_11"))};
    }

    uint8_t TimeOut::score(PlayEvaluator &playEvaluator) noexcept {
        return playEvaluator.getGlobalEvaluation(GlobalEvaluation::TimeOutGameState);
    }

    void TimeOut::calculateInfoForRoles() noexcept {
        const auto xPosition = -4 * control_constants::ROBOT_RADIUS;
        const double distanceToCenterLine = field.getFieldWidth() / 2 - 2 * control_constants::ROBOT_RADIUS;
        const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine : -distanceToCenterLine;

        const std::string formation = "time_out_";
        for (int i = 1; i <= 11; i++) {
            stpInfos[formation + std::to_string(i)].setPositionToMoveTo(Vector2(i * xPosition, yPosition));
        }
    }

    Dealer::FlagMap TimeOut::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        flagMap.insert({"time_out_1", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_2", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_3", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_4", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_5", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_6", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_7", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_8", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_9", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_10", {DealerFlagPriority::REQUIRED, {}}});
        flagMap.insert({"time_out_11", {DealerFlagPriority::REQUIRED, {}}});

        return flagMap;
    }

    const char *TimeOut::getName() { return "Time Out"; }

}  // namespace rtt::ai::stp::play