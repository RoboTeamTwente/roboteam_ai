//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/PenaltyUs.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/PenaltyKeeper.h"
#include "stp/roles/active/PenaltyTaker.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

PenaltyUs::PenaltyUs() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::PenaltyUsGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::PenaltyUsGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::PenaltyTaker>("penalty_taker"), std::make_unique<role::Halt>(role::Halt("halt_0")),
        std::make_unique<role::Halt>(role::Halt("halt_1")),     std::make_unique<role::Halt>(role::Halt("halt_2")),         std::make_unique<role::Halt>(role::Halt("halt_3")),
        std::make_unique<role::Halt>(role::Halt("halt_4")),     std::make_unique<role::Halt>(role::Halt("halt_5")),         std::make_unique<role::Halt>(role::Halt("halt_6")),
        std::make_unique<role::Halt>(role::Halt("halt_7")),     std::make_unique<role::Halt>(role::Halt("halt_8")),
    };
}

Dealer::FlagMap PenaltyUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;  // DONT TOUCH.

    /// Flags that have a factor and a weight linked to it, can be given to a role
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"penalty_taker", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;  // DONT TOUCH.
}

uint8_t PenaltyUs::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::PenaltyUsGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void PenaltyUs::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto penaltyTakerIt = std::find_if(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "penalty_taker"; });

    // We just keep repeating the role until the penalty is over
    if (penaltyTakerIt->get()->finished()) penaltyTakerIt->get()->goToFirstTactic();

    auto ballPos = world->getWorld()->getBall()->get()->position;
    if (stpInfos["penalty_taker"].getRobot()){
        auto robotPos = stpInfos["penalty_taker"].getRobot()->get()->getPos();
        auto enemyKeeper = world->getWorld()->getRobotClosestToPoint(field.getTheirGoalCenter(), world::Team::them);
        // If we're more than 4m away from the goal (and the keeper also isn't closer), we want to slowly kick the ball forward
        if (ballPos.x < (field.getTheirGoalCenter().x - 6.0) && enemyKeeper && enemyKeeper->get()->getDistanceToBall() > 6.0){
            stpInfos["penalty_taker"].setPositionToShootAt(robotPos + (field.getTheirGoalCenter() - robotPos).stretchToLength(3.0));
            stpInfos["penalty_taker"].setShotType(ShotType::TARGET);
            if (penaltyTakerIt != roles.end() && (strcmp(penaltyTakerIt->get()->getCurrentTactic()->getName(), "Formation") == 0)){
                penaltyTakerIt->get()->goToFirstTactic();
            }
        }
        // Else, we want to shoot at their goal
        else {
            stpInfos["penalty_taker"].setPositionToShootAt(field.getTheirGoalCenter() - Vector2(0.0, 0.20));
            stpInfos["penalty_taker"].setShotType(ShotType::MAX);
            if (penaltyTakerIt != roles.end()){
                // if we're  not yet in getBall or orbit kick, skip to that
                if (strcmp(penaltyTakerIt->get()->getCurrentTactic()->getName(), "Get Ball") && strcmp(penaltyTakerIt->get()->getCurrentTactic()->getName(), "Orbit Kick")){
                    penaltyTakerIt->get()->skipToTactic(3);
                }
            }
        }
    }
}

const char* PenaltyUs::getName() { return "Penalty Us"; }

}  // namespace rtt::ai::stp::play
