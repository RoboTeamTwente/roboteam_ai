//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_OUTPUT_H
#define ROBOTEAM_AI_OUTPUT_H

#include <mutex>
#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/utilities/Pause.h>

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
    static std::mutex ballPlacementMutex;
    static std::mutex refMutex;
    static std::mutex showDebugMutex;

    static rtt::Vector2 ballPlacementTarget;
    static bool useRefereeCommands;
    static bool showDebugValuesInTerminal;
    static bool timeOutAtTop;
public:
    static bool isTimeOutAtTop();

public:
    static void setShowDebugValues(bool showDebug);
    static bool getShowDebugValues();
    static bool showDebugLongestTick();
    static bool showDebugTickTimeTaken();
    static bool showDebugNumTreeTimeTaken();
    static bool showDebugNumTreeInfo();
    static bool showFullDebugNumTreeInfo();

    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getBallPlacementTarget();
    static void setBallPlacementTarget(const rtt::Vector2 &ballPlacementTarget);
    static void setTimeOutTop(bool top);

    static const pidVals &getNumTreePid();
    static void setNumTreePid(const pidVals &numTreePid);
    static const pidVals &getForcePid();
    static void setForcePid(const pidVals &forcePid);
    static const pidVals &getBasicPid();
    static void setBasicPid(const pidVals &basicPid);
    static void sendHaltCommand();
};

}
}
}

#endif //ROBOTEAM_AI_OUTPUT_H
