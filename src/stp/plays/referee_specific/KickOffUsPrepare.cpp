//
// Created by jordi on 30-04-20.
//

#include "stp/plays/referee_specific/KickOffUsPrepare.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

    KickOffUsPrepare::KickOffUsPrepare() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("kicker")),
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

    uint8_t KickOffUsPrepare::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::KickOffUsPrepareGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void KickOffUsPrepare::calculateInfoForRoles() noexcept {
        auto width = field.getFieldWidth();
        auto length = field.getFieldLength();

        // Keeper
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

        double defense_line_x = field.getLeftPenaltyX()+0.15;

        // Positions of the KickOffUs formation which will be dealt to the Formation roles in order
        // The "kicker" will go to the ball
        if(stpInfos["kicker"].getRobot() && stpInfos["kicker"].getRobot()->get()->getPos().x < 0){
            Vector2 robotPos = stpInfos["kicker"].getRobot()->get()->getPos();
            Vector2 ballPos = world->getWorld()->getBall()->get()->getPos();
            if((robotPos-ballPos).length() < 0.7){
                stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
            } else if (robotPos.y > 0) {
                stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.6));
            } else {
                stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, -0.6));
            }
        } else {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
        }
        stpInfos["kicker"].setPositionToShootAt(Vector2(4.5, 0));
        stpInfos["formation_1"].setPositionToMoveTo(Vector2(-1, 1));
        stpInfos["formation_2"].setPositionToMoveTo(Vector2(-length / 4, -width / 8));
        stpInfos["formation_3"].setPositionToMoveTo(Vector2(-length / 8, width / 4));
        stpInfos["formation_4"].setPositionToMoveTo(Vector2(-length / 8, -width / 4));
        stpInfos["formation_5"].setPositionToMoveTo(Vector2(defense_line_x, 0.0));
        stpInfos["formation_6"].setPositionToMoveTo(Vector2(defense_line_x, width / 5));
        stpInfos["formation_7"].setPositionToMoveTo(Vector2(defense_line_x, -width / 5));
        stpInfos["formation_8"].setPositionToMoveTo(Vector2(-length / 4, width / 3));
        stpInfos["formation_9"].setPositionToMoveTo(Vector2(-length / 4, -width / 3));
    }

    Dealer::FlagMap KickOffUsPrepare::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::REQUIRED);
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFlag}}});
        flagMap.insert({"formation_1", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
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

    const char *KickOffUsPrepare::getName() { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play