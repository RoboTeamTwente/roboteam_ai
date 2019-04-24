//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTDATA_H
#define ROBOTEAM_AI_SHOTDATA_H

#include "../positionControllers/PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {

struct ShotData : public PosVelAngle {
    bool kick = false;
    bool chip = false;
    double kickSpeed = 0.0;
    int genevaState = 3;

    ShotData() = default;
    ShotData(const Vector2 &p, const Vector2 &v, double a, bool kick, bool chip, double kickSpeed, int genevaState)
    : PosVelAngle(p, v, a), kick(kick), chip(chip), kickSpeed(kickSpeed), genevaState(genevaState)
    { }

    bool operator==(const ShotData &other) {
        return this->pos == other.pos
                 && this->vel == other.vel
                 && this->angle == other.angle
                 && this->kick == other.kick
                 && this->chip == other.chip
                 && this->kickSpeed == other.kickSpeed
                 && this->genevaState == other.genevaState;
    }
};

} // control
} // ai
} // rtt



#endif //ROBOTEAM_AI_SHOTDATA_H
