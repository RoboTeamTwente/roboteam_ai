//
// Created by rolf on 11/12/18.
//

#include "PID.h"

namespace control {

void PID::setParams(double pGainP, double iGainP, double dGainP, double pGainR, double iGainR, double dGainR) {
    pGainPos = pGainP;
    iGainPos = iGainP;
    dGainPos = dGainP;
    pGainRot = pGainR;
    iGainRot = iGainR;
    dGainRot = dGainR;
}

//TODO: test these implementations
// Naive speed computation
Vector2 PID::posControl(Vector2 myPos, Vector2 targetPos) {
    Vector2 err = targetPos - myPos;
    Vector2 P = err*pGainPos;
    I = I + err*iGainPos*timeDif;
    Vector2 D = (err - prevPosErr)*dGainPos/timeDif;
    prevPosErr = err;
    return P + I + D;
}
Vector2 PID::posControl(Vector2 myPos, Vector2 targetPos, Vector2 myVel) {
    Vector2 err = targetPos - myPos;
    Vector2 P = err*pGainPos;
    I = I + err*iGainPos*timeDif;
    Vector2 D = myVel*dGainPos;
    return P + I + D;
}
void PID::initialize(double tickTime) {
    timeDif = tickTime;
    prevPosErr = Vector2(0.0, 0.0);
    I = Vector2(0.0, 0.0);
}

}