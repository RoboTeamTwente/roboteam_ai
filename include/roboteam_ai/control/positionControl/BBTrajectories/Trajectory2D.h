//
// Created by tijmen on 15-12-21.
//

#ifndef RTT_TRAJECTORY2D_H
#define RTT_TRAJECTORY2D_H

#include <vector>
#include "BBTrajectory2D.h"
#include "Trajectory1D.h"
#include "control/positionControl/BBTrajectories/WorldObjects.h"

namespace rtt {

class Trajectory2D {
   public:
    Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double alpha);

    void addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime);

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

}

#endif  // RTT_TRAJECTORY2D_H
