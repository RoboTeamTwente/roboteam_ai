/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include "RefGameState.h"
#include "StrategyManager.h"
#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>

namespace rtt::ai {

class GameStateManager {
   public:
    static void setRefereeData(proto::SSL_Referee refMsg, const rtt::world::World* data);
    static proto::SSL_Referee getRefereeData();
    static GameState getCurrentGameState();
    static bool canEnterDefenseArea(int robotId);
    static bool canMoveOutsideField(int robotId);
    static void forceNewGameState(RefCommand cmd, std::optional<rtt::world::view::BallView> ball);
    static Vector2 getRefereeDesignatedPosition();

   private:
    static proto::SSL_Referee refMsg;
    static StrategyManager strategymanager;
    static std::mutex refMsgLock;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_GAMESTATEMANAGER_HPP
