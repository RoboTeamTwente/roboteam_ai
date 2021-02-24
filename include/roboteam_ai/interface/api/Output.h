//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_OUTPUT_H
#define ROBOTEAM_AI_OUTPUT_H

#include <mutex>

#include "roboteam_utils/Vector2.h"
#include "utilities/GameState.h"
#include "utilities/Pause.h"

namespace rtt::ai::interface {

typedef std::tuple<double, double, double> pidVals;

class Output {
   private:

    static std::mutex markerMutex;
    static std::mutex refMutex;

    static rtt::Vector2 markerPosition;
    static bool useRefereeCommands;
    static bool timeOutAtTop;

    static GameState interfaceGameState;

   public:
    static void sendHaltCommand();

    static void setInterfaceGameState(GameState interfaceGameState);
    static const GameState &getInterfaceGameState();


    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getInterfaceMarkerPosition();
    static void setMarkerPosition(const rtt::Vector2 &ballPlacementTarget);
  static bool isTimeOutAtTop();
  static void setTimeOutTop(bool top);

    static void setRuleSetName(std::string name);
    static void setKeeperId(int id);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_OUTPUT_H
