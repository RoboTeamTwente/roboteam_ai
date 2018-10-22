//
// Created by robzelluf on 10/18/18.
//

#ifndef ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H

#include "../conditions/Condition.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"
#include  <boost/optional.hpp>

namespace rtt {
namespace ai {

class IsRobotClosestToBall : public rtt::ai::Condition {
public:
    IsRobotClosestToBall();
    Status Update();
private:
    ros::NodeHandle n;
};

}
}

#endif //ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
