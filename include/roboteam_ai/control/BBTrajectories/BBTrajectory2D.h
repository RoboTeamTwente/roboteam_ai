//
// Created by rolf on 17-1-20.
//

#ifndef RTT_BBTRAJECTORY2D_H
#define RTT_BBTRAJECTORY2D_H

#include <control/BBTrajectories/BBTrajectory1D.h>
#include <roboteam_utils/Vector2.h>
namespace rtt {
/**
 * @author Rolf
 * @brief Class that computes and stores 2 dimensional bang-bang trajectories.
 */
class BBTrajectory2D {
    public:
        /**
         * @brief Default constructor
         */
        BBTrajectory2D() = default;
        /**
         *
         * @brief Computes a roughly time-optimal bang-bang trajectory. This means that the velocity at the final point will
         * be 0. Any points on the trajectory will always satisfy the maxVel and maxAcc constraints.
         * @param initialPos The initial position to start the trajectory from
         * @param initialVel The initial velocity to start the trajectory from
         * @param finalPos The final position to arrive at.
         * @param maxVel The maximum allowed velocity for this path.
         * @param maxAcc The maximum allowed acceleration allowed for the robot on this path
         */
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc);
        BBTrajectory2D(const Vector2& initialPos, const Vector2& initialVel, const Vector2& finalPos, double maxVel, double maxAcc, double alpha);
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
