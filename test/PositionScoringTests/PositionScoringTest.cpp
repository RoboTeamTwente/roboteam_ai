//
// Created by Alexander de Ranitz on 09-02-22.
//
#include <fstream>

#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "roboteam_utils/Grid.h"
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
ai::stp::gen::ScoredPosition getMaxScore(world::World* world, ai::stp::gen::ScoreProfile profile, int pointsPerMeterX = 10, int pointsPerMeterY = 10) {
    auto length = world->getField().value().getFieldLength();
    auto width = world->getField().value().getFieldWidth();
    auto gridPoints = Grid(-length / 2.0, -width / 2.0, width, length, static_cast<int>(length) * pointsPerMeterX, static_cast<int>(width) * pointsPerMeterY).getPoints();
    auto bestPosition = ai::stp::gen::ScoredPosition{Vector2(), 0};
    for (auto& pointsVec : gridPoints) {
        for (auto& point : pointsVec) {
            // Uncomment this line if you want to ignore invalid positions!
            // if (!ai::FieldComputations::pointIsValidPosition(world->getField().value(), point)) continue;
            auto scoredPosition = ai::stp::PositionScoring::scorePosition(point, profile, world->getField().value(), world);
            if (scoredPosition.score > bestPosition.score) {
                bestPosition = scoredPosition;
            }
        }
    }
    return bestPosition;
}

/// Saves all computed scores in the specified file ([x, y, score\n])
void saveScores(world::World* world, ai::stp::gen::ScoreProfile profile, const std::string& fileName, int pointsPerMeterX = 10, int pointsPerMeterY = 10) {
    auto length = world->getField().value().getFieldLength();
    auto width = world->getField().value().getFieldWidth();
    auto gridPoints = Grid(-length / 2.0, -width / 2.0, width, length, static_cast<int>(length) * pointsPerMeterX, static_cast<int>(width) * pointsPerMeterY).getPoints();
    std::ofstream f;
    f.open(fileName);
    for (auto& pointsVec : gridPoints) {
        for (auto& point : pointsVec) {
            auto scoredPosition = ai::stp::PositionScoring::scorePosition(point, profile, world->getField().value(), world);
            f << scoredPosition.position.x << ", " << scoredPosition.position.y << ", " << (int)scoredPosition.score << "\n";
        }
    }
    f.close();
}

/// Saves robot locations for the specified team in the given file ([x, y])
void saveRobotLocations(world::World* world, world::Team team, const std::string& fileName) {
    std::ofstream f;
    f.open(fileName);
    auto bots = (team == world::Team::us) ? world->getWorld()->getUs() : world->getWorld()->getThem();
    for (auto& bot : bots) {
        f << bot->getPos().x << ", " << bot->getPos().y << "\n";
    }
    f.close();
}

/// Saves the ball location in the specified file [x, y]
void saveBallLocation(world::World* world, const std::string& fileName) {
    std::ofstream f;
    f.open(fileName);
    f << world->getWorld()->getBall()->get()->position.x << ", " << world->getWorld()->getBall()->get()->position.y << "\n";
    f.close();
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

    /// Save all relevant data to visualise- be sure to update the file names and location
    rtt::saveScores(world, scoreProfile, "example_scores_file.txt");
    rtt::saveRobotLocations(world, rtt::world::Team::us, "example_robots_file.txt");
    rtt::saveBallLocation(world, "example_ball_file.txt");

    return 0;
}
