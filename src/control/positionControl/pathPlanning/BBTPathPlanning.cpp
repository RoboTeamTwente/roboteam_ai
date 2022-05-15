//
// Created by martin on 14-5-22.
//

#include "control/positionControl/pathPlanning/BBTPathPlanning.h"

#include <functional>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "interface/widgets/widget.h"

namespace rtt::ai::control {

BBTPathPlanning::BBTPathPlanning(double fieldWidth, double maxVelocity, int robotId, const CollisionDetector& collisionDetector)
    : robotId(robotId), collisionDetector(collisionDetector), fieldWidth(fieldWidth), maxVelocity(maxVelocity) {}

std::vector<BB::PosVelVector> BBTPathPlanning::generateNewPath(const Vector2& initialPos, const Vector2& initialVel, const Vector2& targetPos) const {
    auto trajectory = BB::BBTrajectory2D(initialPos, initialVel, targetPos, maxVelocity, ai::Constants::MAX_ACC_UPPER());
    auto posVel = trajectory.getPosVelVector();

    auto firstCollision = collisionDetector.getFirstCollision(std::span(posVel), robotId, avoidObjects);
    if (!firstCollision.has_value()) return trajectory.getPosVelVector();

    const auto intermediatePoints = generateIntermediatePoints(initialPos);
    auto bestTrajectory = CombinedTrajectory{};

    for (const auto& intermediatePoint : intermediatePoints) {
        auto newTrajectory = bestTrajectoryForIntermediatePoint(initialPos, initialVel, intermediatePoint, targetPos);
        interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, newTrajectory.endTrajectory.getPathApproach(), Qt::red, robotId, interface::Drawing::LINES_CONNECTED);
        if (bestTrajectory.score < newTrajectory.score) continue;
        bestTrajectory = std::move(newTrajectory);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, intermediatePoints, Qt::yellow, robotId, interface::Drawing::CROSSES);
    return extractPath(std::move(bestTrajectory));
}

std::vector<BB::PosVelVector> BBTPathPlanning::extractPath(CombinedTrajectory&& combineTraj) const {
    auto path = std::vector<BB::PosVelVector>();
    const auto initialPosVel = combineTraj.initialTrajectory.getPosVelVector();
    const auto endPosVel = combineTraj.endTrajectory.getPosVelVector();

    path.reserve(combineTraj.cutoffIndex + endPosVel.size());
    path.insert(path.end(), std::make_move_iterator(initialPosVel.begin()), std::make_move_iterator(std::next(initialPosVel.begin(), combineTraj.cutoffIndex)));
    path.insert(path.end(), std::make_move_iterator(endPosVel.begin()), std::make_move_iterator(endPosVel.end()));
    return path;
}

std::vector<Vector2> BBTPathPlanning::generateIntermediatePoints(const Vector2& center) const {
    const auto circlesRadius = {fieldWidth / 8, fieldWidth / 4, fieldWidth / 2};
    auto intermediatePoints = std::vector<Vector2>();
    intermediatePoints.reserve(INTERMEDIATE_POINTS_PER_CIRCLE * circlesRadius.size());

    for (auto radius : circlesRadius) {
        auto pointToRotate = center + Vector2{0, radius};
        for (int i = 0; i < INTERMEDIATE_POINTS_PER_CIRCLE; i++) {
            intermediatePoints.emplace_back(pointToRotate.rotateAroundPoint(i * ANGLE_BETWEEN_INTERMEDIATE_POINTS, center));
        }
    }

    return intermediatePoints;
}

void BBTPathPlanning::updateConstraints(const rtt::world::Field& field, const stp::AvoidObjects newAvoidObjects, double newMaxVelocity) {
    maxVelocity = newMaxVelocity;
    fieldWidth = field.getFieldWidth();
    avoidObjects = newAvoidObjects;
}

CombinedTrajectory BBTPathPlanning::bestTrajectoryForIntermediatePoint(const Vector2& initialPos, const Vector2& initialVel, const Vector2& intermediatePoint,
                                                                       const Vector2& targetPos) const {
    auto bestTrajectory = CombinedTrajectory{BB::BBTrajectory2D(initialPos, initialVel, intermediatePoint, maxVelocity, ai::Constants::MAX_ACC_UPPER())};

    const auto initialPosVel = bestTrajectory.initialTrajectory.getPosVelVector();
    // Checking each step is not necessary and takes too much time
    for (int i = 0; i < initialPosVel.size(); i += 2) {
        const auto& cutoffPoint = initialPosVel[i];
        auto endTrajectory = BB::BBTrajectory2D(cutoffPoint.position, cutoffPoint.velocity, targetPos, maxVelocity, ai::Constants::MAX_ACC_UPPER());
        const auto& endPath = endTrajectory.getPathApproach();

        auto collision = collisionDetector.getFirstCollision(std::span(endPath), robotId, avoidObjects, i);
        if (collision.has_value() && collision->timeStep <= i) break;

        int newScore = scorePath(i + endPath.size(), collision);
        if (newScore >= bestTrajectory.score) continue;

        bestTrajectory.endTrajectory = std::move(endTrajectory);
        bestTrajectory.cutoffIndex = i;
        bestTrajectory.score = newScore;
    }

    return bestTrajectory;
}

int BBTPathPlanning::scorePath(int duration, std::optional<Collision> collision) {
    auto score = duration;
    if (collision.has_value()) {
        score += 500;  // Fixed penalty for collision
        score -= collision->timeStep;
    }

    return score;
}

}  // namespace rtt::ai::control