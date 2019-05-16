/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include "roboteam_msgs/RefereeData.h"
#include "ros/ros.h"
#include "RefGameState.h"

namespace rtt {
namespace ai {

class GameStateManager {
private:
    // we need to store the current and the previous refereedata.
    static roboteam_msgs::RefereeData refMsg;
    static roboteam_msgs::RefereeData previousRefMsg;
    static RefGameState currentRefGameState;
    static RuleSet ruleset;
public:
    static const RuleSet &getRuleset();

public:
    static RefGameState getCurrentRefGameState();
    static void setCurrentRefGameState(RefGameState currentRefGameState);
    static void setCurrentRuleSet(std::string ruleset);
    static void setRefereeData(roboteam_msgs::RefereeData refMsg);
    static roboteam_msgs::RefereeData getRefereeData();
    static roboteam_msgs::RefereeData getPreviousRefereeData();
    static RuleSet getRuleSetByName(std::string name);
};

}//ai
}//rtt
#endif //ROBOTEAM_AI_GAMESTATEMANAGER_HPP
