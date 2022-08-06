#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

// The x position on which we take the penalty
constexpr double PENALTY_MARK_US_X = -2.0;

PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("kicker_formation")),
        std::make_unique<role::Formation>(role::Formation("formation_0")), std::make_unique<role::Formation>(role::Formation("formation_1")),
        std::make_unique<role::Formation>(role::Formation("formation_2")), std::make_unique<role::Formation>(role::Formation("formation_3")),
        std::make_unique<role::Formation>(role::Formation("formation_4")), std::make_unique<role::Formation>(role::Formation("formation_5")),
        std::make_unique<role::Formation>(role::Formation("formation_6")), std::make_unique<role::Formation>(role::Formation("formation_7")),
        std::make_unique<role::Formation>(role::Formation("formation_8"))};
}

uint8_t PenaltyUsPrepare::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::PenaltyUsPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
    // We need at least a keeper, and a kicker positioned behind the ball
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
    stpInfos["kicker_formation"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position - Vector2{0.25, 0.0});

    // During our penalty, all our robots should be behind the ball to not interfere.
    // Create a grid pattern of robots on our side of the field

    // Determine where behind our robots have to stand
    auto ballPosition = world->getWorld()->getBall();
    // If there is no ball, use the default division A penalty mark position
    double ballX = ballPosition.has_value() ? ballPosition.value()->position.x : PENALTY_MARK_US_X;
    double limitX = std::min(ballX, PENALTY_MARK_US_X) - Constants::PENALTY_DISTANCE_BEHIND_BALL();

    // Then, figure out at what interval the robots will stand on a horizontal line
    double horizontalRange = std::fabs(field.getLeftmostX() - limitX);
    double horizontalHalfStep = horizontalRange / (5.0 * 2.0); // 5 robots for stepSize, divided by 2 for half stepSize

    // Lastly, figure out vertical stepSize
    double verticalRange = std::fabs(field.getBottomLeftOurDefenceArea().y - field.getBottommostY());
    double verticalHalfStep = verticalRange / (2.0 * 2.0); // 2 rows, divided by 2 for half stepSize

    double startX = field.getLeftmostX() + horizontalHalfStep;
    double bottomY = field.getBottommostY() + verticalHalfStep;
    double topY = bottomY + 2 * verticalHalfStep;

    const std::string formationPrefix = "formation_";

    /// Bottom row of 5 robots
    for (int i = 0; i < 5; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + i * 2 * horizontalHalfStep, bottomY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.getTheirGoalCenter() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }

    /// Top row of 5 robots
    for (int i = 5; i < 9; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + (i - 5) * 2 * horizontalHalfStep, topY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.getTheirGoalCenter() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }
}

Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker_formation", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char* PenaltyUsPrepare::getName() { return "Penalty Us Prepare"; }
}  // namespace rtt::ai::stp::play
