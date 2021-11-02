//
// Created by maxl on 29-03-21.
//

/// THIS IS MEANT AS A TEMPLATE FOR MAKING PLAYS. DO NOT ADD THIS FILE TO CMAKE.

/// --------- USAGE:
/// 1. Edit the code below the green comments (or ///)
//  2. Dont touch the code with grey comments (or //)
/// 3. Follow the instructions of each comment
/// 4. Remove the unused optional functions

#include "stp/plays/PlayTemplate.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Waller.h"
#include "stp/computations/PositionComputations.h"

namespace rtt::ai::stp::play {
    const char* PlayTemplate::getName() { return "Play Template"; }

    PlayTemplate::PlayTemplate() : Play() {
        /// Evaluations that have to be true to be considered when changing plays.
        startPlayEvaluation.clear(); // DONT TOUCH.
        startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

        /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
        keepPlayEvaluation.clear(); // DONT TOUCH.
        keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

        /// Role creation, the names should be unique. The names are used in the stpInfos-map.
        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>("keeper"),
                std::make_unique<role::Formation>("role_0"),
                std::make_unique<role::Formation>("role_1"),
                std::make_unique<role::Formation>("role_2"),
                std::make_unique<role::Formation>("role_3"),
                std::make_unique<role::Formation>("role_4"),
                std::make_unique<role::Formation>("role_5"),
                std::make_unique<role::Formation>("role_6"),
                std::make_unique<role::Waller>("waller_0"),
                std::make_unique<role::Waller>("waller_1"),
                std::make_unique<role::Waller>("waller_2")};
        initRoles(); // DONT TOUCH.
    }

    Dealer::FlagMap PlayTemplate::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap; // DONT TOUCH.

        /// Flags that have a factor and a weight linked to it, can be given to a role
        Dealer::DealerFlag flag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

        /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"role_0", {DealerFlagPriority::REQUIRED, {flag}}});
        flagMap.insert({"role_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
        flagMap.insert({"role_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
        flagMap.insert({"role_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
        flagMap.insert({"role_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"role_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"role_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"waller_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

        return flagMap; // DONT TOUCH.
    }

    uint8_t PlayTemplate::score(PlayEvaluator& playEvaluator) noexcept {
        calculateInfoForScoredRoles(playEvaluator.getWorld()); /// DISABLE IF NOT USED
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::BallCloseToUs),1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    /// OPTIONAL -> place to calculateInfoForRoles. Make sure not to compute twice.
    void PlayTemplate::calculateInfoForScoredRoles(world::World* world) noexcept {
        stpInfos["role_0"].setPositionToMoveTo(pos::getPosition(stpInfos["role_0"].getPositionToMoveTo(),gen::gridRightMid, gen::SafePosition, world->getField(), world));
    }

    void PlayTemplate::calculateInfoForRoles() noexcept {
        /// Function where are roles get their information, make sure not to compute roles twice.
        calculateInfoForScoredRoles(world); // DONT TOUCH.
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
        stpInfos["keeper"].setPositionToShootAt(field.getTheirGoalCenter());

        stpInfos["role_1"].setPositionToMoveTo(pos::getPosition(stpInfos["role_1"].getPositionToMoveTo(),gen::gridRightTop, gen::GoalShootPosition, field, world));
        stpInfos["role_2"].setPositionToMoveTo(pos::getPosition(stpInfos["role_2"].getPositionToMoveTo(),gen::gridRightBot, gen::OffensivePosition, field, world));
        stpInfos["role_3"].setPositionToMoveTo(pos::getPosition(stpInfos["role_3"].getPositionToMoveTo(),gen::gridMidFieldMid, gen::SafePosition, field, world));
        stpInfos["role_4"].setPositionToMoveTo(pos::getPosition(stpInfos["role_4"].getPositionToMoveTo(),gen::gridMidFieldTop, gen::GoalShootPosition, field, world));
        stpInfos["role_5"].setPositionToMoveTo(pos::getPosition(stpInfos["role_5"].getPositionToMoveTo(),gen::gridMidFieldBot, gen::OffensivePosition, field, world));
        stpInfos["role_6"].setPositionToMoveTo(pos::getPosition(stpInfos["role_6"].getPositionToMoveTo(),gen::gridMidFieldMid, gen::GoalShootPosition, field, world));

        stpInfos["waller_0"].setPositionToMoveTo(pos::getWallPosition(0,3,field,world));
        stpInfos["waller_1"].setPositionToMoveTo(pos::getWallPosition(1,3,field,world));
        stpInfos["waller_2"].setPositionToMoveTo(pos::getWallPosition(2,3,field,world));
    }

    /// OPTIONAL -> place to compute extra evaluations to end a play
    bool PlayTemplate::shouldEndPlay() noexcept {
        return false;
    }

    /// OPTIONAL -> place to save information for next play
    void PlayTemplate::storePlayInfo(PlayInfos& info) noexcept {
        StoreInfo role_0;
        role_0.robotID = stpInfos["role_0"].getRobot()->get()->getId();
        role_0.moveToPosition = stpInfos["role_0"].getPositionToMoveTo();
        info.insert({gen::KeyInfo::isShooter, role_0});
    }
}  // namespace rtt::ai::stp::play
