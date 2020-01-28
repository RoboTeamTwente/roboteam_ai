//
// Created by ratoone on 12-11-19.
//

#ifndef RTT_BASICPATHTRACKING_H
#define RTT_BASICPATHTRACKING_H


#include "utilities/Constants.h"
#include "roboteam_utils/Vector2.h"

namespace rtt::ai::control{

/**
 * Path tracking algorithm. See method computePath for details.
 */
class BasicPathTracking {
private:
    double maxVelocity = rtt::ai::Constants::MAX_VEL();
    // minimum distance needed to consider the current target reached
    double minimumDistance = 2*rtt::ai::Constants::ROBOT_RADIUS();

public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * BasicPathTracking just computes the desired velocity as the difference between the current point
     * and the next one.
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints the path as a list of points
     * @param outputVelocity control velocity that will be outputted to the robot at the current tick
     * @param outputAngle the resulting orientation of the robot at the current tick
     */
    void trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
            std::vector<Vector2> &pathPoints, Vector2 &outputVelocity, double &outputAngle);
};
}

#endif //RTT_BASICPATHTRACKING_H
