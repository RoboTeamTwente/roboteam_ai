//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_PIDTRACKING_H
#define RTT_PIDTRACKING_H

#include "roboteam_utils/Position.h"
#include "utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "interface/api/Output.h"
#include "roboteam_utils/pid.h"

namespace rtt::ai::control{

/**
 * Path tracking algorithm. See method computePath for details.
 */
class PidTracking {
private:
    double maxVelocity = Constants::MAX_VEL();
    double minimumDistance = 2*Constants::ROBOT_RADIUS();

    // the PID controllers on the two axes
    PID xPid = PID(0,0,0,0);
    PID yPid = PID(0,0,0,0);

    void updatePidValues();
public:
    /**
     * The constructor only initializes the maximum velocity to the PID clamping.
     */
    PidTracking();

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
     */
    Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints);
};

}


#endif //RTT_PIDTRACKING_H
