/*
 *
 * Refstate commands are given from the robocup as integers
 *
 */

#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include <proto/messages_robocup_ssl_referee.pb.h>

#include "RefGameState.h"
#include "StrategyManager.h"

namespace rtt::ai {

class GameStateManager {
   public:
    static void setRefereeData(proto::SSL_Referee refMsg, const rtt::world::World* data);
    static proto::SSL_Referee getRefereeData();
    static GameState getCurrentGameState();
    static void forceNewGameState(RefCommand cmd, std::optional<rtt::world::view::BallView> ball);
    static Vector2 getRefereeDesignatedPosition();
    static void updateInterfaceGameState(const char* name);

   private:
    static proto::SSL_Referee refMsg;
    static StrategyManager strategymanager;
    static std::mutex refMsgLock;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_GAMESTATEMANAGER_HPP
