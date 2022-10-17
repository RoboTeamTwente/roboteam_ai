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
    static pidVals numTreePID;
    static pidVals receivePID;
    static pidVals interceptPID;
    static pidVals keeperPID;
    static pidVals keeperInterceptPID;

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

    static bool isTimeOutAtTop();

    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getInterfaceMarkerPosition();
    static void setMarkerPosition(const rtt::Vector2 &ballPlacementTarget);

    static const pidVals &getNumTreePid();
    static void setNumTreePid(const pidVals &numTreePid);
    static const pidVals &getReceivePid();
    static void setReceivePid(const pidVals &receivePid);
    static const pidVals &getInterceptPid();
    static void setInterceptPid(const pidVals &interceptPid);
    static const pidVals &getKeeperPid();
    static void setKeeperPid(const pidVals &keeperPid);
    static const pidVals &getKeeperInterceptPid();
    static void setKeeperInterceptPid(const pidVals &keeperInterceptPid);

    static void setRuleSetName(std::string name);
    static void setKeeperId(int id);
    static void setBallPlacerId(int id);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_OUTPUT_H
