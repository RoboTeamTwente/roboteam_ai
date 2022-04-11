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

PassInfo PassComputations::calculatePass(gen::ScoreProfile profile, const rtt::world::World* world, const world::Field& field, bool keeperCanPass) {
    PassInfo passInfo;  // Struct used to store the information needed to execute the pass

    auto us = world->getWorld()->getUs();
    auto ballLocation = world->getWorld()->getBall()->get()->getPos();

    // Find which robot is keeper (bot closest to goal if there was not a keeper yet), store its id, and erase from us
    passInfo.keeperId = getKeeperId(us, world, field);
    if (!keeperCanPass) std::erase_if(us, [passInfo](auto& bot) { return bot->getId() == passInfo.keeperId; });
    if (keeperCanPass) RTT_DEBUG("Keeper can pass!");
    if (!keeperCanPass) RTT_DEBUG("Keeper cant pass!");

    // Find which robot should be the passer, store its id and location, and erase from us
    passInfo.passerId = getPasserId(ballLocation, us, world);
    auto passerIt = std::find_if(us.begin(), us.end(), [passInfo](auto& bot) { return bot->getId() == passInfo.passerId; });
    auto passerLocation = passerIt->get()->getPos();
    us.erase(passerIt);

    // If we don't have any robots that could be a receiver, just shoot towards their goal
    if (us.empty()){
        passInfo.passLocation = field.getTheirGoalCenter();
        return passInfo;
    }

    // This is a vector with the locations of all robots that could act as a receiver (ie all robots except the keeper and the passer)
    std::vector<Vector2> possibleReceiverLocations;
    possibleReceiverLocations.reserve(us.size());
    for (auto& robot : us) {
        possibleReceiverLocations.emplace_back(robot->getPos());
    }

    auto possiblePassLocationsVector = getPassGrid(field).getPoints();
    for (auto& pointVector : possiblePassLocationsVector) {
        for (auto& point : pointVector) {
            if (pointIsValidPassLocation(point, ballLocation, possibleReceiverLocations, passerLocation, field, world)) {
                gen::ScoredPosition scoredPosition = PositionScoring::scorePosition(point, profile, field, world);
                if (scoredPosition.score > passInfo.passScore) {
                    passInfo.passScore = scoredPosition.score;
                    passInfo.passLocation = scoredPosition.position;
                    passInfo.receiverId = world->getWorld()->getRobotClosestToPoint(passInfo.passLocation, us).value()->getId();
                }
            }
        }
    }

    if (passInfo.passScore == 0) {
        // If no good pass is found, pass to the robot furthest in the field
        passInfo.passLocation = *std::max_element(possibleReceiverLocations.begin(), possibleReceiverLocations.end(), [](auto& p1, auto& p2) { return p1.x > p2.x; });
    }

    return passInfo;
}

Grid PassComputations::getPassGrid(const world::Field& field) {
    double gridWidth = field.getFieldWidth();
    double gridLength = field.getFieldLength();
    int numPoints = 9;
    return Grid(-gridLength / 2, -gridWidth / 2, gridWidth, gridLength, numPoints, numPoints);  // 81 points spread over the whole field
}

bool PassComputations::pointIsValidPassLocation(Vector2 point, Vector2 ballLocation, const std::vector<Vector2>& possibleReceiverLocations, Vector2 passerLocation,
                                                const world::Field& field, const world::World* world) {
    constexpr double MINIMUM_PASS_DISTANCE = 1.0;  // This can be dribbled instead of passed
    if (point.dist(ballLocation) < MINIMUM_PASS_DISTANCE) return false;
    constexpr double MINIMUM_LINE_OF_SIGHT = 10.0;  // The minimum LoS to be a valid pass, otherwise, the pass will go into an enemy robot
    if (PositionScoring::scorePosition(point, gen::LineOfSight, field, world).score < MINIMUM_LINE_OF_SIGHT) return false;
    if (!FieldComputations::pointIsValidPosition(field, point, 2 * control_constants::ROBOT_RADIUS)) return false;

    // Pass is valid if the above conditions are met and there is a robot whose travel time is smaller than the balls travel time (i.e. the robot can actually receive the ball)
    auto ballTravelTime = calculateBallTravelTime(ballLocation, passerLocation, point);
    return std::any_of(possibleReceiverLocations.begin(), possibleReceiverLocations.end(),
                       [&](auto& robotPos) { return calculateRobotTravelTime(robotPos, point) < ballTravelTime; });
}

int PassComputations::getPasserId(Vector2 ballLocation, const std::vector<world::view::RobotView>& possibleRobots, const world::World* world) {
    auto passer = world->getWorld()->getRobotClosestToPoint(ballLocation, possibleRobots);
    if (passer) {
        return passer->get()->getId();
    }
    return -1;
}

int PassComputations::getKeeperId(const std::vector<world::view::RobotView>& possibleRobots, const world::World* world, const world::Field& field) {
    auto keeperId = GameStateManager::getCurrentGameState().keeperId;
    auto keeperIt = std::find_if(possibleRobots.begin(), possibleRobots.end(), [keeperId](const auto& bot) { return bot->getId() == keeperId; });
    if (keeperIt != possibleRobots.end()) {
        return (*keeperIt)->getId();
    }
    auto keeper = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), possibleRobots);
    if (keeper) {
        return keeper->get()->getId();
    }
    return -1;  // Should never reach this point unless there are no robots
}

// TODO: use BBT for this
double PassComputations::calculateRobotTravelTime(Vector2 robotPosition, Vector2 targetPosition) { return robotPosition.dist(targetPosition) * 2; }

// TODO: create better approximation
double PassComputations::calculateBallTravelTime(Vector2 ballPosition, Vector2 passerLocation, Vector2 targetPosition) {
    auto travelTime = calculateRobotTravelTime(passerLocation, ballPosition - (passerLocation - ballPosition).stretchToLength(control_constants::ROBOT_RADIUS));
    auto rotateTime = (ballPosition - passerLocation).toAngle().shortestAngleDiff(targetPosition - ballPosition) / (control_constants::ANGLE_RATE * 2);
    double ballSpeed = control::ControlUtils::determineKickForce(ballPosition.dist(targetPosition), ShotType::PASS);
    auto ballTime = ballPosition.dist(targetPosition) / ballSpeed;
    return travelTime + rotateTime + ballTime;
}

bool PassComputations::pathHasAnyRobots(Line passLine, std::vector<Vector2> robotLocations) {
    Tube passTube = Tube(passLine.v1, passLine.v2, control_constants::ROBOT_RADIUS);
    return std::any_of(robotLocations.begin(), robotLocations.end(), [&](const auto& location) { return passTube.contains(location); });
}
}  // namespace rtt::ai::stp::computations