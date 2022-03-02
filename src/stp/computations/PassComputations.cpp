//
// Created by maxl on 11-02-21.
//

#include <roboteam_utils/Grid.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>
#include <stp/computations/PositionScoring.h>

#include "control/ControlUtils.h"
#include "roboteam_utils/Tube.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::computations {

gen::ScoredPosition PassComputations::calculatePassLocation(Vector2 ballLocation, const std::vector<Vector2>& robotLocations, Vector2 passerLocation, gen::ScoreProfile profile,
                                                            const rtt::world::World* world, const world::Field& field) {
    double gridWidth = field.getFieldWidth();
    double gridLength = field.getFieldLength();
    int numPoints = 9;
    gen::ScoredPosition bestPassLocation;
    bestPassLocation.score = 0;
    if (robotLocations.empty()) {
        RTT_WARNING("No possible receivers!")
        return bestPassLocation;
    }
    // TODO: create a better passingGrid (more points and/or better spread)
    auto passGrid = Grid(-gridLength / 2, -gridWidth / 2, gridWidth, gridLength, numPoints, numPoints);  // 81 points spread over the whole field
    for (auto& pointVector : passGrid.getPoints()) {
        for (auto& point : pointVector) {
            if (point.dist(ballLocation) < 1) continue; // Do not pass less than 1m far- this can be dribbled
            if (!FieldComputations::pointIsValidPosition(field, point, 2 * control_constants::ROBOT_RADIUS)) continue;
            if (PositionScoring::scorePosition(point, gen::LineOfSight, field, world).score < 10) continue;  // Need minimum LoS to be a valid pass. Avoid passing into enemies
            for (auto robotLocation : robotLocations) {
                if (calculateRobotTravelTime(robotLocation, point) < calculateBallTravelTime(ballLocation, passerLocation, point)) {
                    gen::ScoredPosition scoredPosition = PositionScoring::scorePosition(point, profile, field, world);
                    if (scoredPosition.score > bestPassLocation.score) {
                        bestPassLocation = scoredPosition;
                    }
                    break;
                }
            }
        }
    }
    if (bestPassLocation.score == 0) {
        RTT_DEBUG("No good pass found! Passing to robot furthest in field.")
        bestPassLocation.position = *std::max_element(robotLocations.begin(), robotLocations.end(), [](auto& p1, auto& p2) { return p1.x > p2.x; });
    }
    return bestPassLocation;
}

// TODO: use BBT for this
double PassComputations::calculateRobotTravelTime(Vector2 robotPosition, Vector2 targetPosition) { return robotPosition.dist(targetPosition) * 2; }

// TODO: create better approximation
double PassComputations::calculateBallTravelTime(Vector2 ballPosition, Vector2 passerLocation, Vector2 targetPosition) {
    auto travelTime = calculateRobotTravelTime(passerLocation, ballPosition - (passerLocation - ballPosition).stretchToLength(control_constants::ROBOT_RADIUS));
    auto rotateTime = (ballPosition - passerLocation).toAngle().shortestAngleDiff(targetPosition - passerLocation) / (control_constants::ANGLE_RATE * 2);
    double ballSpeed = control::ControlUtils::determineKickForce(ballPosition.dist(targetPosition), ShotType::PASS);
    auto ballTime = ballPosition.dist(targetPosition) / ballSpeed;
    return travelTime + rotateTime + ballTime;
}

bool PassComputations::pathHasAnyRobots(Line passLine, std::vector<Vector2> robotLocations) {
    Tube passTube = Tube(passLine.v1, passLine.v2, control_constants::ROBOT_RADIUS);
    return std::any_of(robotLocations.begin(), robotLocations.end(), [&](const auto& location) { return passTube.contains(location); });
}
}  // namespace rtt::ai::stp::computations