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
    static pidVals shotControllerPID;
    public:
        static const pidVals &getShotControllerPID();
        static void setShotControllerPID(const pidVals &shotControllerPID);
    private:
    static pidVals keeperPID;
    static pidVals keeperInterceptPID;
    static pidVals ballHandlePID;

    static std::mutex markerMutex;
    static std::mutex refMutex;
    static std::mutex showDebugMutex;

    static rtt::Vector2 markerPosition;
    static bool useRefereeCommands;
    static bool showDebugValuesInTerminal;
    static bool timeOutAtTop;

    static GameState interfaceGameState;
public:
    static void setInterfaceGameState(GameState interfaceGameState);

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
        static bool showCoachTimeTaken();
        static bool showFullDebugNumTreeInfo();

    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getInterfaceMarkerPosition();
    static void setMarkerPosition(const rtt::Vector2 &ballPlacementTarget);
    static void setTimeOutTop(bool top);

    static const pidVals &getNumTreePid();
    static void setNumTreePid(const pidVals &numTreePid);
    static const pidVals &getBasicPid();
    static void setBasicPid(const pidVals &basicPid);
    static const pidVals &getKeeperPid();
    static void setKeeperPid(const pidVals &keeperPid);
    static const pidVals &getKeeperInterceptPid();
    static void setKeeperInterceptPid(const pidVals &keeperInterceptPid);
    static const pidVals &getBallHandlePid();
    static void setBallHandlePid(const pidVals &ballHandlePid);
    static void sendHaltCommand();

    static void setKeeperTree(std::string name);
    static void setStrategyTree(std::string name);
    static void setRuleSetName(std::string name);
    static void setKeeperId(int id);
};

}
}
}

#endif //ROBOTEAM_AI_OUTPUT_H
