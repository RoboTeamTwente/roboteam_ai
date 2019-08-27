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
#include "StrategyManager.h"

namespace rtt {
namespace ai {

class GameStateManager {
public:
    static void setRefereeData(roboteam_msgs::RefereeData refMsg);
    static roboteam_msgs::RefereeData getRefereeData();
    static GameState getCurrentGameState();
    static bool canEnterDefenseArea(int robotId);
    static bool canMoveOutsideField(int robotId);
    static void forceNewGameState(RefCommand cmd);
private:
    static roboteam_msgs::RefereeData refMsg;
    static StrategyManager strategymanager;

};

}//ai
}//rtt
#endif //ROBOTEAM_AI_GAMESTATEMANAGER_HPP
