/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include "messages_robocup_ssl_referee.pb.h"
#include "RefGameState.h"
#include "StrategyManager.h"

namespace rtt {
namespace ai {

class GameStateManager {
public:
    static void setRefereeData(roboteam_proto::SSL_Referee refMsg);
    static roboteam_proto::SSL_Referee getRefereeData();
    static GameState getCurrentGameState();
    static bool canEnterDefenseArea(int robotId);
    static bool canMoveOutsideField(int robotId);
    static void forceNewGameState(RefCommand cmd);
    static Vector2 getRefereeDesignatedPosition();
private:
    static roboteam_proto::SSL_Referee refMsg;
    static StrategyManager strategymanager;
    static std::mutex refMsgLock;

};

}//ai
}//rtt
#endif //ROBOTEAM_AI_GAMESTATEMANAGER_HPP
