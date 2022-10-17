//
// Created by rolf on 26-09-20.
//
#include "control/positionControl/BBTrajectories/BBTrajectory1D.h"

#include <cmath>

namespace rtt::BB {
double BBTrajectory1D::fullBrakePos(double pos, double vel, double accMax) {
    double acc = vel <= 0 ? accMax : -accMax;
    double t = -vel / acc;       // (inverted) time needed to break to zero velocity
    return pos + 0.5 * vel * t;  // position after breaking
}

double BBTrajectory1D::accelerateBrakePos(double pos0, double vel0, double vel1, double accMax) {
    double acc1;
    double acc2;
    if (vel1 >= vel0) {
        acc1 = accMax;
        acc2 = -accMax;
    } else {
        acc1 = -accMax;
        acc2 = accMax;
    }
    double t1 = (vel1 - vel0) / acc1;                 // time to reach vel1
    double pos1 = pos0 + (0.5 * (vel0 + vel1) * t1);  // position at which we reach vel1
    double t2 = -vel1 / acc2;                         // time to max break from vel1 to 0
    return pos1 + (0.5 * vel1 * t2);                  // position we stop at after initiating maximal break at pos1
}

void BBTrajectory1D::triangularProfile(double startPos, double startVel, double endPos, double maximumAcc, bool invertedSign) {
    double acc = invertedSign ? maximumAcc : -maximumAcc;
    // compute the final time difference at which we switch from accelerating to breaking
    double sq = (acc * (endPos - startPos) + 0.5 * startVel * startVel) / (acc * acc);
    double brakeTime = sq > 0 ? sqrt(sq) : 0;
    double topVel = acc * brakeTime;
    double switchTime = (topVel - startVel) / acc;
    double switchPos = startPos + (startVel + topVel) * 0.5 * switchTime;
    updatePart(0, switchTime, acc, startVel, startPos);
    updatePart(1, switchTime + brakeTime, -acc, topVel, switchPos);
    numParts = 2;
}

void BBTrajectory1D::updatePart(int index, double tEnd, double acc, double vel, double pos) {
    parts[index].acc = acc;
    parts[index].startPos = pos;
    parts[index].startVel = vel;
    parts[index].tEnd = tEnd;
}

void BBTrajectory1D::trapezoidalProfile(double startPos, double startVel, double maximumVel, double endPos, double maximumAcc) {
    double acc1 = startVel > maximumVel ? -maximumAcc : maximumAcc;
    double acc3 = maximumVel > 0 ? -maximumAcc : maximumAcc;
    double t1 = (maximumVel - startVel) / acc1;
    double t3 = -maximumVel / acc3;
    double startCoastPos = startPos + 0.5 * (startVel + maximumVel) * t1;
    double endCoastPos = endPos - 0.5 * maximumVel * t3;
    double t2 = (endCoastPos - startCoastPos) / maximumVel;

    updatePart(0, t1, acc1, startVel, startPos);
    updatePart(1, t1 + t2, 0, maximumVel, startCoastPos);
    updatePart(2, t1 + t2 + t3, acc3, maximumVel, endCoastPos);
    numParts = 3;
}

void BBTrajectory1D::generateTrajectory(double startPos, double startVel, double endPos, double maximumVel, double maximumAcc) {
    // Store call so we can review it later:
    finalPos = endPos;
    maxVel = maximumVel;

    double brakePos = fullBrakePos(startPos, startVel, maximumAcc);
    if (brakePos <= endPos) {
        // Check if we need triangular profile or trapezoidal:
        double accBrakePos = accelerateBrakePos(startPos, startVel, maximumVel, maximumAcc);
        if (accBrakePos >= endPos) {
            triangularProfile(startPos, startVel, endPos, maximumAcc, true);
        } else {
            trapezoidalProfile(startPos, startVel, maximumVel, endPos, maximumAcc);
        }
    } else {
        // similar to above but with slightly mirrored logic
        double accBrakePos = accelerateBrakePos(startPos, startVel, -maximumVel, maximumAcc);
        if (accBrakePos <= endPos) {
            triangularProfile(startPos, startVel, endPos, maximumAcc, false);
        } else {
            trapezoidalProfile(startPos, startVel, -maximumVel, endPos, maximumAcc);
        }
    }
}

BBTrajectory1D::BBTrajectory1D(double startPos, double startVel, double endPos, double maximumVel, double maximumAcc)
    : maxVel{maximumVel} {
    generateTrajectory(startPos, startVel, endPos, maximumVel, maximumAcc);
}

BBPosVelAcc BBTrajectory1D::getValues(double t) const {
    double trajTime = fmax(0, t);
    BBTrajectoryPart piece = parts[0];
    if (trajTime >= getTotalTime()) {
        // The time is not on the trajectory so we just return the last known element
        return BBPosVelAcc(finalPos, 0, 0);  // can also be computed from parts if necessary
    }
    // we step through the parts and try to find the relevant part on which the time is.
    double tPieceStart = 0;
    for (int i = 0; i < numParts; ++i) {
        piece = parts[i];
        if (trajTime <= parts[i].tEnd) {
            break;
        }
        tPieceStart = parts[i].tEnd;
    }
    double tPiece = trajTime - tPieceStart;
    // extrapolate the state given the information we have.
    return BBPosVelAcc(piece.startPos + piece.startVel * tPiece + 0.5 * piece.acc * tPiece * tPiece, piece.startVel + piece.acc * tPiece, piece.acc);
}

double BBTrajectory1D::getTotalTime() const { return parts[numParts - 1].tEnd; }

double BBTrajectory1D::getAcceleration(double t) const {
    double trajTime = fmax(0, t);
    if (trajTime >= getTotalTime()) {
        return 0;
    }
    BBTrajectoryPart piece = parts[0];
    // we step through the parts and try to find the relevant part on which the time is.
    for (int i = 0; i < numParts; ++i) {
        piece = parts[i];
        if (trajTime <= parts[i].tEnd) {
            break;
        }
    }
    return piece.acc;
}

double BBTrajectory1D::getVelocity(double t) const {
    double trajTime = fmax(0, t);
    BBTrajectoryPart piece = parts[0];
    if (trajTime >= getTotalTime()) {
        // The time is not on the trajectory so we just return the last known element
        return 0;
    }
    // we step through the parts and try to find the relevant part on which the time is.
    double tPieceStart = 0;
    for (int i = 0; i < numParts; ++i) {
        piece = parts[i];
        if (trajTime <= parts[i].tEnd) {
            break;
        }
        tPieceStart = parts[i].tEnd;
    }
    double tPiece = trajTime - tPieceStart;
    // extrapolate the state given the information we have.
    return piece.startVel + piece.acc * tPiece;
}

double BBTrajectory1D::getPosition(double t) const {
    double trajTime = fmax(0, t);
    BBTrajectoryPart piece = parts[0];
    if (trajTime >= getTotalTime()) {
        return finalPos;
    }
    // we step through the parts and try to find the relevant part on which the time is.
    double tPieceStart = 0;
    for (int i = 0; i < numParts; ++i) {
        piece = parts[i];
        if (trajTime <= parts[i].tEnd) {
            break;
        }
        tPieceStart = parts[i].tEnd;
    }
    double tPiece = trajTime - tPieceStart;
    // extrapolate the state given the information we have.
    return piece.startPos + piece.startVel * tPiece + 0.5 * piece.acc * tPiece * tPiece;
}

bool BBTrajectory1D::inLastPart(double t) const { return t >= parts[numParts - 2].tEnd; }

std::vector<BB::BBTrajectoryPart> BBTrajectory1D::getParts() {
    std::vector<BB::BBTrajectoryPart> partsVector;
    partsVector.reserve(numParts);
    for (int i = 0; i < numParts; i++) {
        partsVector.push_back(parts[i]);
    }
    return partsVector;
}
}  // namespace rtt::BB