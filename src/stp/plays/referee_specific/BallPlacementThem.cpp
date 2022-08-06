//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_3")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_4")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_5")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_6")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_7")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_8")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("waller_9")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("harasser"))};
}

uint8_t BallPlacementThem::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallPlacementThemGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_9", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"harasser", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void BallPlacementThem::calculateInfoForRoles() noexcept {
    calculateInfoForWallers();
    calculateInfoForKeeper();
    calculateInfoForHarasser();
}

void BallPlacementThem::calculateInfoForWallers() noexcept {

    constexpr auto wallerNames = std::array{"waller_1", "waller_2", "waller_3", "waller_4", "waller_5", "waller_6", "waller_7", "waller_8", "waller_9"};
    auto activeWallerNames = std::vector<std::string>{};
    for (auto name : wallerNames) {
        if (stpInfos[name].getRobot().has_value()) activeWallerNames.emplace_back(name);
    }
    for (int i = 0; i < activeWallerNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        const auto side = i % 2 == 0 ? 1 : -1;
        auto& wallerStpInfo = stpInfos[activeWallerNames[i]];

        wallerStpInfo.setPositionToMoveTo(
            Vector2(FieldComputations::getDefenseArea(field, true, 0, 0)[2].x + 2 * control_constants::ROBOT_RADIUS, side * 1.7 * (i+1) * control_constants::ROBOT_RADIUS));
        wallerStpInfo.setAngle((Vector2{0, 0} - field.getOurGoalCenter()).angle());
    }
}

void BallPlacementThem::calculateInfoForHarasser() noexcept {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto targetPos = placementPos + (field.getOurGoalCenter() - placementPos).stretchToLength(control_constants::AVOID_BALL_DISTANCE);
    targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, world->getWorld()->getBall().value()->position, field);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setAngle((placementPos - field.getOurGoalCenter()).toAngle());
}

void BallPlacementThem::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
}

const char* BallPlacementThem::getName() { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play
