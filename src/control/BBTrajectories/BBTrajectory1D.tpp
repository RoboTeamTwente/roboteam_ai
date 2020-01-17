//
// Created by rolf on 15-01-20.
//

#include "control/BBTrajectories/BBTrajectory1D.h"
#include <cmath>
template<class num>
num BBTrajectory1D<num>::fullBrakePos(num pos, num vel, num accMax) {
    num acc = vel <= 0 ? accMax : -accMax;
    num t = -vel / acc; // time needed to break to zero velocity
    return pos + 0.5 * vel * t; // position after breaking
}
template<class num>
num BBTrajectory1D<num>::accelerateBrakePos(num pos0, num vel0, num vel1, num accMax) {
    num acc1;
    num acc2;
    if (vel1 >= vel0) {
        acc1 = accMax;
        acc2 = -accMax;
    } else {
        acc1 = -accMax;
        acc2 = accMax;
    }
    num t1 = (vel1 - vel0) / acc1; // time to reach vel1
    num pos1 = pos0 + (0.5 * (vel0 + vel1) * t1); //position at which we reach vel1
    num t2 = -vel1 / acc2; // time to max break from vel1 to 0
    return pos1 + (0.5 * vel1 * t2); // position we stop at after initiating maximal break at pos1

}
template<class num>
void
BBTrajectory1D<num>::triangularProfile(num initialPos, num initialVel, num finalPos, num maxAcc, bool invertedSign) {
    num brakeTime;
    num topVel;
    num switchTime;
    num acc;
    num switchPos;

    if (invertedSign) {
        //compute the final time difference at which we switch from accelerating to breaking
        num sq = (maxAcc * (finalPos - initialPos) + 0.5 * initialVel * initialVel) / (maxAcc * maxAcc);
        if (sq > 0) {
            brakeTime = sqrt(sq);
        } else {
            brakeTime = 0;
        }
        topVel = maxAcc * brakeTime;
        switchTime = (topVel - initialVel) / maxAcc;
        acc = maxAcc;
        switchPos = initialPos + (initialVel + topVel) * 0.5 * switchTime;
    } else {
        num sq = (maxAcc * (initialPos - finalPos) + 0.5 * initialVel * initialVel) / (maxAcc * maxAcc);
        if (sq > 0) {
            brakeTime = sqrt(sq);
        } else {
            brakeTime = 0;
        }
        topVel = -maxAcc * brakeTime;
        switchTime = (topVel - initialVel) / -maxAcc;
        acc = -maxAcc;
        switchPos = initialPos + (initialVel + topVel) * 0.5 * switchTime;
    }
    updatePart(0, switchTime, acc, initialVel, initialPos);
    updatePart(1, switchTime + brakeTime, -acc, topVel, switchPos);
    numParts = 2;
}
template<class num>
void BBTrajectory1D<num>::updatePart(int index, num tEnd, num acc, num vel, num pos) {
    parts[index].acc = acc;
    parts[index].startPos = pos;
    parts[index].startVel = vel;
    parts[index].tEnd = tEnd;
}
template<class num>
void BBTrajectory1D<num>::trapezoidalProfile(num initialPos, num initialVel, num maxVel, num finalPos, num maxAcc) {
    num acc1;
    num acc3;

    if (initialVel > maxVel) {
        acc1 = -maxAcc;
    } else {
        acc1 = maxAcc;
    }

    if (maxVel > 0) {
        acc3 = -maxAcc;
    } else {
        acc3 = maxAcc;
    }

    num t1 = (maxVel - initialVel) / acc1;
    num t3 = -maxVel / acc3;
    num startCoastPos = initialPos + 0.5 * (initialVel + maxVel) * t1;
    num endCoastPos = finalPos - 0.5 * maxVel * t3;
    num t2 = (endCoastPos - startCoastPos) / maxVel;

    updatePart(0, t1, acc1, initialVel, initialPos);
    updatePart(1, t1 + t2, 0, maxVel, startCoastPos);
    updatePart(2, t1 + t2 + t3, acc3, maxVel, endCoastPos);
    numParts = 3;

}
template<class num>
void BBTrajectory1D<num>::generateTrajectory(num initialPos, num initialVel, num finalPos, num maxVel, num maxAcc) {
    num brakePos = fullBrakePos(initialPos, initialVel, maxAcc);
    if (brakePos <= finalPos) {
        //Check if we need triangular profile or trapezoidal:
        num accBrakePos = accelerateBrakePos(initialPos, initialVel, maxVel, maxAcc);
        if (accBrakePos >= finalPos) {
            triangularProfile(initialPos, initialVel, finalPos, maxAcc, true);
        } else {
            trapezoidalProfile(initialPos, initialVel, maxVel, finalPos, maxAcc);
        }
    } else {
        //similar to above
        num accBrakePos = accelerateBrakePos(initialPos, initialVel, -maxVel, maxAcc);
        if (accBrakePos <= finalPos) {
            triangularProfile(initialPos, initialVel, finalPos, maxAcc, false);
        } else {
            trapezoidalProfile(initialPos, initialVel, -maxVel, finalPos, maxAcc);
        }
    }

}
template<class num>
BBTrajectory1D<num>::BBTrajectory1D(num initialPos, num finalPos, num initialVel, num maxAcc, num maxVel) :
        initialPos{initialPos},
        initialVel{initialVel},
        finalPos{finalPos},
        maxAcc{maxAcc},
        maxVel{maxVel} {
    generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc);
}
template<class num>
PosVelAcc<num> BBTrajectory1D<num>::getValues(num t) {
    num trajTime = max(0, t);
    part *piece = parts[0];
    if (trajTime >= getTotalTime()) {
        //The time is not on the trajectory so we just return the last known element
        return PosVelAcc(0, 0, finalPos);//can also be computed from parts if necessary
    }
    num tPieceStart = 0;
    for (int i = 0; i < numParts; ++i) {
        piece = parts[i];
        if (trajTime <= parts[i].tEnd) {
            break;
        }
        tPieceStart = parts[i].tEnd;
    }
    num tPiece = trajTime - tPieceStart;
    return PosVelAcc<num>(piece->acc,
                     piece->startVel + piece->acc * tPiece,
                     piece->startPos + piece->startVel * tPiece + 0.5 * piece->acc * tPiece * tPiece);
}
template<class num>
num BBTrajectory1D<num>::getTotalTime() {
    return parts[numParts - 1].tEnd;
}
