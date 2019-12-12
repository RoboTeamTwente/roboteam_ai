/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include "roboteam_proto/messages_robocup_ssl_referee.pb.h"
#include "RefGameState.h"
#include "StrategyManager.h"

namespace rtt::ai {

class GameStateManager {
public:
    static void setRefereeData(proto::SSL_Referee refMsg);
    static proto::SSL_Referee getRefereeData();
    static GameState getCurrentGameState();
    static bool canEnterDefenseArea(int robotId);
    static bool canMoveOutsideField(int robotId);
    static void forceNewGameState(RefCommand cmd);
    static Vector2 getRefereeDesignatedPosition();
private:
    static proto::SSL_Referee refMsg;
    static StrategyManager strategymanager;
    static std::mutex refMsgLock;

};

}//rtt
#endif //ROBOTEAM_AI_GAMESTATEMANAGER_HPP
