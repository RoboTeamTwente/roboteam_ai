//
// Created by rolf on 17-1-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2D.h"
namespace rtt {
BBTrajectory2D::BBTrajectory2D(Vector2 initialPos, Vector2 initialVel, Vector2 finalPos,
        float maxVel, float maxAcc, float alpha)
        :
        x{BBTrajectory1D<float>(initialPos.x, finalPos.x, initialVel.x, maxAcc*cosf(alpha), maxVel*cosf(alpha))},
        y{BBTrajectory1D<float>(initialPos.y, finalPos.y, initialVel.y, maxAcc*sinf(alpha), maxVel*sinf(alpha))} {
}
}