//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_OUTPUT_H
#define ROBOTEAM_AI_OUTPUT_H

#include <mutex>

#include "roboteam_utils/Vector2.h"
#include "utilities/GameState.h"

namespace rtt::ai::interface {


class Output {
   private:

    static std::mutex refMutex;

    static bool useRefereeCommands;

    static GameState interfaceGameState;

   public:
    static void setInterfaceGameState(GameState interfaceGameState);
    static const GameState &getInterfaceGameState();


    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);



    static void setRuleSetName(std::string name);
    static void setKeeperId(int id);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_OUTPUT_H
