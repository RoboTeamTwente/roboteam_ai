//
// Created by tijmen on 16-12-21.
//

#ifndef RTT_TRAJECTORY1D_H
#define RTT_TRAJECTORY1D_H

#include "BBTrajectory1D.h"

namespace rtt {

class Trajectory1D {
   public:
    void addTrajectory(const std::vector<BB::BBTrajectoryPart> &newParts, double addFromTime);

    /**
     * @brief Gets the position at time t
     * @param t time to get position at
     * @return Position at time t
     */
    [[nodiscard]] double getPosition(double t) const;

    /**
     * @brief Gets the velocity at time t
     * @param t time to get velocity at
     * @return Velocity at time t
     */
    [[nodiscard]] double getVelocity(double t) const;

    /**
     * @brief Gets the acceleration at time t
     * @param t time to get acceleration at.
     * @return Acceleration at time t
     */
    [[nodiscard]] double getAcceleration(double t) const;

    /**
     * @brief Gets the total trajectory time.
     * @return Total time of the trajectory to end point
     */
    [[nodiscard]] double getTotalTime() const;

    std::vector<BB::BBTrajectoryPart> parts;
    // m
    double finalPos;    // m
    // m/s
    // m/s^2
    double maxVel;      // m/s
};

}  // namespace rtt

#endif  // RTT_TRAJECTORY1D_H
