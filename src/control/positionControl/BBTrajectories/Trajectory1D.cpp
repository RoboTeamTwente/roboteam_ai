//
// Created by tijmen on 16-12-21.
//

#include "control/positionControl/BBTrajectories/Trajectory1D.h"

#include <cmath>

namespace rtt {

void Trajectory1D::addTrajectory(const std::vector<BB::BBTrajectoryPart>& newParts, double addFromTime) {
    for (size_t i = 0; i < parts.size(); i++) {
        if (addFromTime <= parts[i].tEnd) {
            parts[i].tEnd = addFromTime;
            parts.erase(parts.begin() + 1 + i, parts.end());
            break;
        }
    }

    for (BB::BBTrajectoryPart newPart : newParts) {
        newPart.tEnd += addFromTime;
        parts.push_back(newPart);
    }
}

double Trajectory1D::getAcceleration(double t) const {
    double trajTime = fmax(0, t);
    if (trajTime >= getTotalTime()) {
        return 0;
    }
    BB::BBTrajectoryPart piece = parts[0];
    // we step through the parts and try to find the relevant part on which the time is.
    for (const auto& part : parts) {
        piece = part;
        if (trajTime <= part.tEnd) {
            break;
        }
    }
    return piece.acc;
}

double Trajectory1D::getVelocity(double t) const {
    double trajTime = fmax(0, t);
    BB::BBTrajectoryPart piece = parts[0];
    if (trajTime >= getTotalTime()) {
        // The time is not on the trajectory so we just return the last known element
        return 0;
    }
    // we step through the parts and try to find the relevant part on which the time is.
    double tPieceStart = 0;
    for (const auto& part : parts) {
        piece = part;
        if (trajTime <= part.tEnd) {
            break;
        }
        tPieceStart = part.tEnd;
    }
    double tPiece = trajTime - tPieceStart;
    // extrapolate the state given the information we have.
    return piece.startVel + piece.acc * tPiece;
}

double Trajectory1D::getPosition(double t) const {
    double trajTime = fmax(0, t);
    BB::BBTrajectoryPart piece = parts[0];
    if (trajTime >= getTotalTime()) {
        return finalPos;
    }
    // we step through the parts and try to find the relevant part on which the time is.
    double tPieceStart = 0;
    for (const auto& part : parts) {
        piece = part;
        if (trajTime <= part.tEnd) {
            break;
        }
        tPieceStart = part.tEnd;
    }
    double tPiece = trajTime - tPieceStart;
    // extrapolate the state given the information we have.
    return piece.startPos + piece.startVel * tPiece + 0.5 * piece.acc * tPiece * tPiece;
}

double Trajectory1D::getTotalTime() const { return parts[parts.size() - 1].tEnd; }

}  // namespace rtt