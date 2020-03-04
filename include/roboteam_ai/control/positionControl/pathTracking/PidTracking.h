//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_PIDTRACKING_H
#define RTT_PIDTRACKING_H

#include "control/positionControl/PositionControlUtils.h"
#include "interface/api/Output.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/pid.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

/**
 * Path tracking algorithm. See method computePath for details.
 */
class PidTracking {
   private:
    static constexpr double MAX_VELOCITY = Constants::MAX_VEL();

    // PID controllers for each robot
    std::unordered_map<int, std::pair<PID, PID>> pidMapping = {};

    // updates the PID parameters from the UI
    void updatePidValuesFromInterface();

   public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * PidTracking applied a PID to the difference of positions, to obtain a velocity. The values are
     * taken from the interface.
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints the path as a list of points
     * @param outputVelocity control velocity that will be outputted to the robot at the current tick
     * @param outputAngle the resulting orientation of the robot at the current tick
     * @param robotId the robotId of the controlled robot - used only for state based tracking (like PID)
     */
    Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId = 0);
};
}  // namespace rtt::ai::control

#endif  // RTT_PIDTRACKING_H
