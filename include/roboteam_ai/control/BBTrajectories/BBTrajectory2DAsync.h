//
// Created by rolf on 18-01-20.
//

#ifndef RTT_BBTRAJECTORY2DASYNC_H
#define RTT_BBTRAJECTORY2DASYNC_H
#include "BBTrajectory2D.h"
namespace rtt {
template<class num>
class BBTrajectory2DAsync {
    private:
        BBTrajectory1D<num> x;
        BBTrajectory1D<num> y;
        num rotation;
        Vector2 startPosition;
    public:
        BBTrajectory2DAsync() = default;
        BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, const LineSegment &line);
        void generateTrajectory( const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, num alpha) noexcept;
        void generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                num maxVel, num maxAcc, const LineSegment &line) noexcept;
        Vector2 getPosition(num t) const;
        Vector2 getVelocity(num t) const;
        Vector2 getAcceleration(num t) const;
        std::vector<Vector2> visCurve() const;

};

}
#include "src/control/BBTrajectories/BBTrajectory2DAsync.tpp"
#endif //RTT_BBTRAJECTORY2DASYNC_H
