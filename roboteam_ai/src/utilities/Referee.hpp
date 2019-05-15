/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_REFEREE_HPP
#define ROBOTEAM_AI_REFEREE_HPP

#include "roboteam_msgs/RefereeData.h"
#include "ros/ros.h"
#include "RefGameState.h"

namespace rtt {
namespace ai {

class Referee {
private:
    // we need to store the current and the previous refereedata.
    static roboteam_msgs::RefereeData refMsg;
    static roboteam_msgs::RefereeData previousRefMsg;
    static RefGameState currentRefGameState;
public:
    static const RefGameState &getCurrentRefGameState();

    static void setCurrentRefGameState(const RefGameState &currentRefGameState);

public:
    static void setRefereeData(roboteam_msgs::RefereeData refMsg);
    static roboteam_msgs::RefereeData getRefereeData();
    static roboteam_msgs::RefereeData getPreviousRefereeData();


};

}//ai
}//rtt
#endif //ROBOTEAM_AI_REFEREE_HPP
