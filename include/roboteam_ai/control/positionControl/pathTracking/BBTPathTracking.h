//
// Created by tijmen on 27-10-21.
//

#ifndef RTT_BBTPATHTRACKING_H
#define RTT_BBTPATHTRACKING_H

#include <span>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/pid.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

enum UpdatePath {
    DONT_UPDATE,
    UPDATE_TARGET_CHANGED,
    UPDATE_TARGET_REACHED,

    UPDATE_COLLISION_DETECTED,
};

inline std::ostream& operator<<(std::ostream& os, const UpdatePath updatePath) {
    switch (updatePath) {
        case UpdatePath::DONT_UPDATE:
            os << "DONT_UPDATE";
            break;
        case UpdatePath::UPDATE_TARGET_CHANGED:
            os << "UPDATE_TARGET_CHANGED";
            break;
        case UpdatePath::UPDATE_TARGET_REACHED:
            os << "UPDATE_TARGET_REACHED";
            break;
        case UpdatePath::UPDATE_COLLISION_DETECTED:
            os << "UPDATE_COLLISION_DETECTED";
            break;
    }
    return os;
}

/**
 * Path tracker for single robot.
 */
class BBTPathTracking {
   private:
    static constexpr size_t STEPS_AHEAD = 1;
    const int robotId;
    const CollisionDetector& collisionDetector;

    PID xPID;
    PID yPID;

    std::vector<BB::PosVelVector> path;
    std::span<const BB::PosVelVector> remainingPath;

   public:
    BBTPathTracking(int robotId, const CollisionDetector& collisionDetector);

    /**
     * \brief Determine if the path needs to be recomputed (e.g. when target has changed)
     * @param currentPos Current position of the robot
     * @param targetPos Target position of the robot from STP
     * @param avoidObjects Objects deemed to be obstacles for collision detection
     *
     * @returns UpdatePath::DONT_UPDATE (i.e. 0) if the path is up to date else ret_val > 0
     */
    [[nodiscard]] UpdatePath shouldUpdatePath(const Vector2& currentPos, const Vector2& targetPos, const stp::AvoidObjects& avoidObjects);

    /**
     * \brief Update the path
     * @param newPath New path to be used
     */
    void updatePath(std::vector<BB::PosVelVector>&& newPath);

    /**
     * \brief Computes and returns next position and angle to be send to the robot
     * @param currentPos Current position of the robot
     * @param currentVel Current velocity of the robot
     * @param pidType PID type to be used (TODO: What is pid type?)
     * @returns Next position and velocity to be send to the robot
     */
    [[nodiscard]] Position trackPathForwardAngle(const Vector2& currentPosition, const Vector2& currentVelocity, stp::PIDType pidType);

    /**
     * \brief Returns readonly view of the remaining path
     */
    [[nodiscard]] std::span<const BB::PosVelVector> getRemainingPath();
};

}  // namespace rtt::ai::control

#endif  // RTT_BBTPATHTRACKING_H
