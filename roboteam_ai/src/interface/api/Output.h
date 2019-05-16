//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_OUTPUT_H
#define ROBOTEAM_AI_OUTPUT_H

#include <mutex>
#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/utilities/Pause.h>
#include "../../utilities/GameState.h"

namespace rtt {
namespace ai {
namespace interface {

    typedef std::tuple<double, double, double> pidVals;


class Output {
private:
    static pidVals basicPID;
    static pidVals numTreePID;
    static pidVals forcePID;

    static std::mutex pidMutex;
    static std::mutex markerMutex;
    static std::mutex refMutex;
    static std::mutex showDebugMutex;

    static rtt::Vector2 markerPosition;
    static bool useRefereeCommands;
    static bool showDebugValuesInTerminal;
    static bool timeOutAtTop;

    static GameState interfaceGameState;
public:
    static void setInterfaceGameState(const GameState &interfaceGameState);

public:
    static const GameState &getInterfaceGameState();

public:
    static bool isTimeOutAtTop();
    static void setShowDebugValues(bool showDebug);
    static bool getShowDebugValues();
    static bool showDebugLongestTick();
    static bool showDebugTickTimeTaken();
    static bool showDebugNumTreeTimeTaken();
    static bool showDebugNumTreeInfo();
    static bool showFullDebugNumTreeInfo();

    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getInterfaceMarkerPosition();
    static void setMarkerPosition(const rtt::Vector2 &ballPlacementTarget);
    static void setTimeOutTop(bool top);

    static const pidVals &getNumTreePid();
    static void setNumTreePid(const pidVals &numTreePid);
    static const pidVals &getForcePid();
    static void setForcePid(const pidVals &forcePid);
    static const pidVals &getBasicPid();
    static void setBasicPid(const pidVals &basicPid);
    static void sendHaltCommand();

    static void setKeeperTree(std::string name);
    static void setStrategyTree(std::string name);
    static void setRuleSetName(std::string name);
    static void setUseKeeper(bool useKeeper);
};

}
}
}

#endif //ROBOTEAM_AI_OUTPUT_H
