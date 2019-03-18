//
// Created by thijs on 19-2-19.
//

#ifndef ROBOTEAM_AI_POSVELANGLE_H
#define ROBOTEAM_AI_POSVELANGLE_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>

namespace rtt {
namespace ai {
namespace control {

class PosVelAngle {
    public:
        Vector2 pos = Vector2();
        Vector2 vel = Vector2();
        Angle angle = Angle();

        PosVelAngle() = default;
        PosVelAngle(const Vector2 &p, const Vector2 &v, double a) :
            pos(p), vel(v), angle(a) {}

        bool isZero() {
            return *this == PosVelAngle();
        }

        bool operator==(const PosVelAngle &other) {
            return this->pos == other.pos && this->vel == other.vel && this->angle == other.angle;
        }
};

}
}
}

#endif //ROBOTEAM_AI_POSVELANGLE_H
