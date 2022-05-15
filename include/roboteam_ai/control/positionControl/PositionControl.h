//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include <queue>
#include <roboteam_utils/RobotCommands.hpp>

#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/pathPlanning/BBTPathPlanning.h"
#include "control/positionControl/pathTracking/BBTPathTracking.h"
#include "control/positionControl/pathTracking/PidTracking.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

struct PositionControlCommand {
    RobotCommand robotCommand;
    bool isOccupied = false;
};

/**
 * The main position control class. Use this for your robot position control
 * requirements.
 */
class PositionControl {
   private:
    CollisionDetector collisionDetector;
    std::unordered_map<int, std::pair<BBTPathPlanning, BBTPathTracking>> pathControllers;

   public:
    /**
     * @brief Generates a collision-free trajectory and tracks it.
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity that the robot is allowed to have
     * @param pidType The desired PID type (intercept, regular, keeper etc.)
     * @param avoidObjects The objects that the robot should avoid
     * @return A PositionControlCommand containing the robot command and path flags
     */
    PositionControlCommand computeAndTrackTrajectory(const rtt::world::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity, Vector2 targetPosition,
                                                     double maxRobotVelocity, stp::PIDType pidType, stp::AvoidObjects avoidObjects);

    /**
     * @brief Returns reference to the collision detector.
     */
    CollisionDetector &getCollisionDetector();
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
