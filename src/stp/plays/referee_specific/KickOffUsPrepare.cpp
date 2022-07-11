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
        std::make_unique<role::Formation>(role::Formation("keeper")),         std::make_unique<role::BallAvoider>(role::BallAvoider("kicker")),
        std::make_unique<role::Formation>(role::Formation("defender_left")),  std::make_unique<role::Formation>(role::Formation("defender_mid")),
        std::make_unique<role::Formation>(role::Formation("defender_right")), std::make_unique<role::Formation>(role::Formation("midfielder_left")),
        std::make_unique<role::Formation>(role::Formation("midfielder_mid")), std::make_unique<role::Formation>(role::Formation("midfielder_right")),
        std::make_unique<role::Formation>(role::Formation("attacker_left")),  std::make_unique<role::Formation>(role::Formation("attacker_mid")),
        std::make_unique<role::Formation>(role::Formation("attacker_right"))};
}

uint8_t KickOffUsPrepare::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::KickOffUsPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void KickOffUsPrepare::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    // The "kicker" will go to the ball
    if (stpInfos["kicker"].getRobot() && stpInfos["kicker"].getRobot()->get()->getPos().x < 0) {
        Vector2 robotPos = stpInfos["kicker"].getRobot()->get()->getPos();
        Vector2 ballPos = world->getWorld()->getBall()->get()->position;
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

    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Defenders
    double defense_line_x = field.getLeftPenaltyX() + control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN;
    stpInfos["defender_left"].setPositionToMoveTo(Vector2(defense_line_x, width / 5));
    stpInfos["defender_mid"].setPositionToMoveTo(Vector2(defense_line_x, 0.0));
    stpInfos["defender_right"].setPositionToMoveTo(Vector2(defense_line_x, -width / 5));

    // Midfielders
    stpInfos["midfielder_left"].setPositionToMoveTo(Vector2(-length / 5, width / 3));
    stpInfos["midfielder_mid"].setPositionToMoveTo(Vector2(-length / 4, 0));
    stpInfos["midfielder_right"].setPositionToMoveTo(Vector2(-length / 5, -width / 3));

    // Attackers
    stpInfos["attacker_left"].setPositionToMoveTo(Vector2(-1, 1));
    stpInfos["attacker_mid"].setPositionToMoveTo(Vector2(-length / 8, 0));
    stpInfos["attacker_right"].setPositionToMoveTo(Vector2(-length / 12, -width / 4));
}

Dealer::FlagMap KickOffUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});
    flagMap.insert({"defender_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_left", {DealerFlagPriority::HIGH_PRIORITY, {kickerFirstPriority}}});
    flagMap.insert({"attacker_mid", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

const char* KickOffUsPrepare::getName() { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play