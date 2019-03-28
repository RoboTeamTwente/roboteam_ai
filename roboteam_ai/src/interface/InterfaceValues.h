//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_INTERFACEVALUES_H
#define ROBOTEAM_AI_INTERFACEVALUES_H

#include <mutex>
#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/utilities/Pause.h>

namespace rtt {
namespace ai {
namespace interface {

class InterfaceValues {
    private:
        static double numTreePosP;
        static double numTreePosI;
        static double numTreePosD;

        static double numTreeVelP;
        static double numTreeVelI;
        static double numTreeVelD;

        static std::mutex pidMutex;
        static std::mutex ballPlacementMutex;
        static std::mutex refMutex;
        static std::mutex showDebugMutex;

        static rtt::Vector2 ballPlacementTarget;
        static bool useRefereeCommands;
        static bool showDebugValuesInTerminal;

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

        static double getNumTreePosP();
        static void setNumTreePosP(double numTreePP);
        static double getNumTreePosI();
        static void setNumTreePosI(double numTreePI);
        static double getNumTreePosD();
        static void setNumTreePosD(double numTreePD);

        static double getNumTreeVelP();
        static void setNumTreeVelP(double numTreeVP);
        static double getNumTreeVelI();
        static void setNumTreeVelI(double numTreeVI);
        static double getNumTreeVelD();
        static void setNumTreeVelD(double numTreeVD);

        static void sendHaltCommand();
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
