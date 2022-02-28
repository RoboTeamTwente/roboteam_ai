//
// Created by Alexander de Ranitz on 28-02-22.
//
#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "roboteam_utils/Grid.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PositionScoring.h"

namespace rtt {
/// Generates a random world and field of 12mx9m with 22 robots and a ball
world::World* generateWorld() {
    auto field = testhelpers::FieldHelper::generateField();
    auto protoWorld = testhelpers::WorldHelper::getWorldMsg(11, 11, true, field);
    auto const& [_, world] = world::World::instance();
    world->updateWorld(protoWorld);
    world->updateField(field);
    return world;
}

/**
 * Gets the max score on the field
 * @param world the current world (should contain a valid field)
 * @param profile the profile to be used when scoring the positions
 * @param pointsPerMeterX the amount of points to generate per meter in the x direction
 * @param pointsPerMeterY the amount of points to generate per meter in the y direction
 * @return scoredPosition with the highest score of all considered points
 */
ai::stp::gen::ScoredPosition getMaxScore(world::World* world, ai::stp::gen::ScoreProfile profile, int pointsPerMeterX = 50, int pointsPerMeterY = 50) {
    auto length = world->getField().value().getFieldLength();
    auto width = world->getField().value().getFieldWidth();
    auto gridPoints = Grid(-length / 2.0, -width / 2.0, width, length, static_cast<int>(length) * pointsPerMeterX, static_cast<int>(width) * pointsPerMeterY).getPoints();
    auto bestPosition = ai::stp::gen::ScoredPosition{Vector2(), 0};
    for (auto& pointsVec : gridPoints) {
        for (auto& point : pointsVec) {
            if (!ai::FieldComputations::pointIsValidPosition(world->getField().value(), point)) continue;
            auto scoredPosition = ai::stp::PositionScoring::scorePosition(point, profile, world->getField().value(), world);
            if (scoredPosition.score > bestPosition.score) {
                bestPosition = scoredPosition;
            }
        }
    }
    return bestPosition;
}

}  // namespace rtt
int main(int argc, char* argv[]) {
    // rtt::ai::Constants::init();
    auto world = rtt::generateWorld();

    /// Set the profile to be used when scoring positions
    auto scoreProfile = rtt::ai::stp::gen::AttackingPass;

    /// Get and print the best position and its score
    auto bestPos = rtt::getMaxScore(world, scoreProfile);
    std::cout << "Best position = (" << bestPos.position.x << ", " << bestPos.position.y << ") with score " << (int)bestPos.score << std::endl;

    auto length = world->getField().value().getFieldLength();
    auto width = world->getField().value().getFieldWidth();
    auto searchGrid = rtt::Grid(-length / 2.0, -width / 2.0, width, length, 9, 9);
    auto foundPos = rtt::ai::stp::PositionComputations::getPosition(std::nullopt, searchGrid, scoreProfile, world->getField().value(), world);
    std::cout << "Found position = (" << foundPos.position.x << ", " << foundPos.position.y << ") with score " << (int)foundPos.score << std::endl;

    std::cout << "Distance between positions = " << bestPos.position.dist(foundPos.position) << "m" << std::endl;
    std::cout << "Difference in score = " << 100.0 - (float)foundPos.score / (float)bestPos.score * 100.0 << "%" << std::endl;
    return 0;
}
