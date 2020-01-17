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
        BBTrajectory2D() = default;
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, float maxVel, float maxAcc);
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, float maxVel, float maxAcc,float alpha);
        void generateTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, float maxVel, float maxAcc, float alpha);
        void generateSyncedTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, float maxVel, float maxAcc);
        BBTrajectory1D<double> x;
        BBTrajectory1D<double> y;

};
}
#endif //RTT_BBTRAJECTORY2D_H
