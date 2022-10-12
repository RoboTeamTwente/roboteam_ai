//
// Created by umer on 12-10-22.
//
#include "stp/plays/offensive/Pass.h"
#include <roboteam_utils/LineSegment.h>
#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/constants/ControlConstants.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "world/views/RobotView.hpp"


namespace rtt::ai::stp::play{
    Pass::Pass() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        startPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);

        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::Passer>(role::Passer("passer")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver")),
                std::make_unique<role::Formation>(role::Formation("halt_1")),
                std::make_unique<role::Formation>(role::Formation("halt_2")),
                std::make_unique<role::Formation>(role::Formation("halt_3")),
                std::make_unique<role::Formation>(role::Formation("halt_4")),
                std::make_unique<role::Formation>(role::Formation("halt_5")),
                std::make_unique<role::Formation>(role::Formation("halt_6")),
                std::make_unique<role::Formation>(role::Formation("halt_7")),
                std::make_unique<role::Formation>(role::Formation("halt_8")),
        }
    }




    uint8_t Pass::score(const rtt::world::Field& field) noexcept{
        passInfo = stp::computations::PassComputations::calculatePass(gen::Pass, world, field);

        if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

        return stp::computations::PassComputations::scorePass(passInfo, world, field);
    }

    Dealer::FlagMap Pass::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
        flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {closeToBallFlag}, passInfo.passerId}});
        flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
        flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        return flagMap;
    }

    void Pass::calculateInfoForRoles() noexcept {
        /// Keeper
        stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

        /// Passer and Receiver
        stpInfos["passer"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position);
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
        stpInfos["passer"].setShotType(ShotType::PASS);





    }

    void Pass::calculateInfoForDefenders() noexcept {

    }

    void Pass::calculateInfoForBlocker() noexcept {

    }

    void Pass::calculateInfoForMidfielders() noexcept {

    }

    void Pass::calculateInfoForAttackers() noexcept {

    }

    bool Pass::ballKcked() {

    }

    bool Pass::shouldEndPlay() noexcept {

    }

    const char* Pass::getName() {
        return "Pass";
    }

} // namespace rtt::ai::stp::play