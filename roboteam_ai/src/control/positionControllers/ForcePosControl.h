//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_FORCEPOSCONTROL_H
#define ROBOTEAM_AI_FORCEPOSCONTROL_H

#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class ForcePosControl : public PosController {
public:
    explicit ForcePosControl() = default;
    PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) override;
    Vector2 calculateForces(const RobotPtr &robot, const Vector2 &targetPos, double forceRadius) const;

protected:
    PosVelAngle calculateForcePosVelAngle(RobotPtr robot, Vector2 &targetPos);

private:
    const double FORCE_WEIGHT_US = 1.0;
    const double FORCE_WEIGHT_THEM = 2.0;
    const double FORCE_WEIGHT_BALL = 1.0;
    const double FORCE_WEIGHT_FIELD_SIDES = 1.0;
    const float POINT_IN_FIELD_MARGIN = 0.5;
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_FORCEPOSCONTROL_H
