//
// Created by rolf on 22-10-18.
//

#ifndef ROBOTEAM_AI_CANREACHPOINT_HPP
#define ROBOTEAM_AI_CANREACHPOINT_HPP

#include "ros/ros.h"
#include "Condition.h"
#include "../../src/utilities/World.h"

namespace rtt {
namespace ai {
/**
* \class CanReachPoint
* \brief Descr: Checks whether a robot can reach a point within a given time limit.
*    Params:
*   - ROBOT_ID:
*       Type: Int
*       Descr: The robot to check for
*   - xGoal:
*       Type: Double
*       Descr: The x-coordinate of the target location
*   - yGoal:
*       Type: Double
*       Descr: The y-coordinate of the target location
*    - timeLimit:
*       Type: Double
*       Descr: The maximum allowed time to reach the target location
*/
class CanReachPoint : public Condition {
    public:
        CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard);
        double estimateTimeToPoint(Vector2 currentPos, Vector2 currentVel, Vector2 targetPos);
        Status update() override;
        std::string node_name() override { return "CanReachPoint"; }
    private:
        //See to do in CPP file. Move these to some sorts of constants file.
        /*
        double maxAcc = 2;
        double maxDec = 4;
        double maxVel = 2;
        double posPGain = 2.0;
        double decelerationDistance = 0.2; //
         */
};
}
}

#endif //ROBOTEAM_AI_CANREACHPOINT_HPP
