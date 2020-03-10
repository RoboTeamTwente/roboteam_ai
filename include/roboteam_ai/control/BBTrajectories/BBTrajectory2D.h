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
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc);
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc,double alpha);
        void generateTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc, double alpha);
        void generateSyncedTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc);
        Vector2 getPosition(double t) const;
        Vector2 getVelocity(double t) const;
        Vector2 getAcceleration(double t) const;
        std::vector<Vector2> visCurve() const;
        BBTrajectory1D x;
        BBTrajectory1D y;

};
}
#endif //RTT_BBTRAJECTORY2D_H
