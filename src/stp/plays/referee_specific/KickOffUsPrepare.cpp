//
// Created by jordi on 30-04-20.
//

#include "stp/plays/referee_specific/KickOffUsPrepare.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffUsPrepare::KickOffUsPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")), std::make_unique<role::BallAvoider>(role::BallAvoider("kicker")),
        std::make_unique<role::Formation>(role::Formation("receiver")), std::make_unique<role::Formation>(role::Formation("defender_0"))};
}

uint8_t KickOffUsPrepare::score(PlayEvaluator &playEvaluator) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::KickOffUsPrepareGameState), 1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();  // DONT TOUCH.
}

void KickOffUsPrepare::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    // The "kicker" will go to the ball
    if (stpInfos["kicker"].getRobot() && stpInfos["kicker"].getRobot()->get()->getPos().x < 0) {
        Vector2 robotPos = stpInfos["kicker"].getRobot()->get()->getPos();
        Vector2 ballPos = world->getWorld()->getBall()->get()->getPos();
        if ((robotPos - ballPos).length() < 0.7) {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
        } else if (robotPos.y > 0) {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.6));
        } else {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, -0.6));
        }
    } else {
        stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
    }

    auto fieldLength = field.getFieldLength();
    auto fieldWidth = field.getFieldWidth();

    // TODO: set position to go to based on where the kicker will pass to in kickoffus
    stpInfos["receiver"].setPositionToMoveTo(Vector2(-0.15 * fieldLength, -0.25 * fieldWidth));

    // TODO: set defender position in smarter way
    stpInfos["defender_0"].setPositionToMoveTo(Vector2(-0.15 * fieldLength, 0.25 * fieldWidth));
}

Dealer::FlagMap KickOffUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFlag}}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char *KickOffUsPrepare::getName() { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play