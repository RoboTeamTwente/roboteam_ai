//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/KickOffUs.h"

#include "stp/roles/passive/Formation.h"
#include "stp/roles/active/PassReceiver.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

    KickOffUs::KickOffUs() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::KickOffUsGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::KickOffUsGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>("keeper"),
                std::make_unique<role::Formation>("passer"),
                std::make_unique<role::PassReceiver>("receiver"),
                std::make_unique<role::Halt>("halt_0"),
                std::make_unique<role::Halt>("halt_1"),
                std::make_unique<role::Halt>("halt_2"),
                std::make_unique<role::Halt>("halt_3"),
                std::make_unique<role::Halt>("halt_4"),
                std::make_unique<role::Halt>("halt_5"),
                std::make_unique<role::Halt>("halt_6"),
                std::make_unique<role::Halt>("halt_7")};
    }

    uint8_t KickOffUs::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::KickOffUsGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void KickOffUs::calculateInfoForRoles() noexcept {
        // Keeper
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
        stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

        // Kicker
        stpInfos["passer"].setPositionToShootAt(field.getTheirGoalCenter());
        stpInfos["passer"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos());
        stpInfos["passer"].setShotType(ShotType::PASS);
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);

        // Receiver
        stpInfos["receiver"].setPositionToMoveTo(Vector2{-1.0, 1.0});
    }

    Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::FlagInstruction kickerFlag(Flag::CLOSEST_TO_BALL, Priority::REQUIRED);
        Dealer::FlagInstruction closeToBallFlag(Flag::CLOSE_TO_BALL, Priority::HIGH_PRIORITY);

        flagMap.insert({"keeper", {Priority::KEEPER, {}}});
        flagMap.insert({"passer", {Priority::REQUIRED, {kickerFlag}}});
        flagMap.insert({"receiver", {Priority::REQUIRED, {closeToBallFlag}}});
        flagMap.insert({"halt_0", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_1", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_2", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_3", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_4", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_5", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_6", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"halt_7", {Priority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    const char *KickOffUs::getName() { return "Kick Off Us"; }

}  // namespace rtt::ai::stp::play
