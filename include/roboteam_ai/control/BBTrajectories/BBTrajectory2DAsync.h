//
// Created by rolf on 18-01-20.
//

#ifndef RTT_BBTRAJECTORY2DASYNC_H
#define RTT_BBTRAJECTORY2DASYNC_H
#include "BBTrajectory2D.h"
namespace rtt {
template<class num>
class BBTrajectory2DAsync {
    public:
        BBTrajectory2DAsync() = default;
        BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, const Vector2 &lineDirection);
        void generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, num alpha) noexcept;
        void generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, const Vector2 &lineDirection) noexcept;
        BBTrajectory1D<num> x;
        BBTrajectory1D<num> y;

};

}
#include "src/control/BBTrajectories/BBTrajectory2DAsync.tpp"
#endif //RTT_BBTRAJECTORY2DASYNC_H
