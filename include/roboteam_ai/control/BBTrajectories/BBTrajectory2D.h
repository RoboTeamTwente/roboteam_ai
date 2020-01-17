//
// Created by rolf on 17-1-20.
//

#ifndef RTT_BBTRAJECTORY2D_H
#define RTT_BBTRAJECTORY2D_H

#include <control/BBTrajectories/BBTrajectory1D.h>
#include <roboteam_utils/Vector2.h>
namespace rtt {
class BBTrajectory2D {
    public:
        BBTrajectory2D(Vector2 initialPos, Vector2 initialVel, Vector2 finalPos, float maxVel, float maxAcc,float alpha);
        BBTrajectory1D<float> x;
        BBTrajectory1D<float> y;

};
}
#endif //RTT_BBTRAJECTORY2D_H
