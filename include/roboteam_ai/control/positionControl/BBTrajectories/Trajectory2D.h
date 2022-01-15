//
// Created by tijmen on 15-12-21.
//

#ifndef RTT_TRAJECTORY2D_H
#define RTT_TRAJECTORY2D_H

#include <vector>

#include "Trajectory1D.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class Trajectory2D {
   public:
    /**
     * @brief Default constructor
     */
    Trajectory2D() = default;

    Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc);

    void addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime);

    /**
     * @brief Approaches the Trajectory by dividing the path in points which are separated by timeStep seconds
     * @param timeStep time between pathpoints
     * @return
     */
    [[nodiscard]] std::vector<Vector2> getPathApproach(double timeStep) const;

    /**
     * @brief Returns a vector with all the velocities (Vector2) at specified timeSteps
     */
    [[nodiscard]] std::vector<Vector2> getVelocityVector(double timeStep) const;

    /**
     * @brief Get the position in the trajectory at time t
     * @param t Given time.
     * @return Position at time t
     */
    [[nodiscard]] Vector2 getPosition(double t) const;

    /**
     * @brief Get the velocity in the trajectory at time t
     * @param t Given time.
     * @return Velocity at time t
     */
    [[nodiscard]] Vector2 getVelocity(double t) const;

    /**
     * @brief Get the acceleration in the trajectory at time t
     * @param t Given time.
     * @return Acceleration at time t
     */
    [[nodiscard]] Vector2 getAcceleration(double t) const;

    /**
     * @brief Gets the total trajectory time.
     * @return Total time of the trajectory to end point
     */
    [[nodiscard]] double getTotalTime() const;

   private:
    Trajectory1D x;
    Trajectory1D y;
};

}  // namespace rtt

#endif  // RTT_TRAJECTORY2D_H
