//
// Created by rolf on 17-1-20.
//

#ifndef RTT_BBTRAJECTORY2D_H
#define RTT_BBTRAJECTORY2D_H

#include <control/BBTrajectories/BBTrajectory1D.h>
#include <roboteam_utils/Vector2.h>
namespace rtt {
template<class num>
class BBTrajectory2D {
    public:
        BBTrajectory2D() = default;
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, num maxVel, num maxAcc) noexcept;
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, num maxVel, num maxAcc,num alpha) noexcept;
        void generateTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, num maxVel, num maxAcc, num alpha) noexcept;
        void generateSyncedTrajectory(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, num maxVel, num maxAcc) noexcept;
        BBTrajectory1D<num> x;
        BBTrajectory1D<num> y;

};
}
#include "src/control/BBTrajectories/BBTrajectory2D.cpp"
#endif //RTT_BBTRAJECTORY2D_H
