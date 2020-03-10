//
// Created by rolf on 18-01-20.
//

#ifndef RTT_BBTRAJECTORY2DASYNC_H
#define RTT_BBTRAJECTORY2DASYNC_H

#include "BBTrajectory2D.h"

namespace rtt {
class LineSegment;

/**
 * TODO: for now not used.
 */
class BBTrajectory2DAsync {
    private:
        BBTrajectory1D x;
        BBTrajectory1D y;
        double rotation;
        Vector2 startPosition;
    public:
        BBTrajectory2DAsync() = default;
        BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                double maxVel, double maxAcc, const LineSegment &line);
        void generateTrajectory( const Vector2 &initialVel, const Vector2 &finalPos,
                double maxVel, double maxAcc, double alpha);
        void generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
                double maxVel, double maxAcc, const LineSegment &line);
        Vector2 getPosition(double t) const;
        Vector2 getVelocity(double t) const;
        Vector2 getAcceleration(double t) const;
        std::vector<Vector2> visCurve() const;
};
}
#endif //RTT_BBTRAJECTORY2DASYNC_H
