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
        static double luthPosP;
        static double luthPosI;
        static double luthPosD;

        static double luthVelP;
        static double luthVelI;
        static double luthVelD;

        static std::mutex PIDMutex;
        static std::mutex BallPlacementMutex;
        static std::mutex RefMutex;
        static std::mutex ShowDebugMutex;

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

        static bool usesRefereeCommands();
        static void setUseRefereeCommands(bool useRefereeCommands);
        static const rtt::Vector2 &getBallPlacementTarget();
        static void setBallPlacementTarget(const rtt::Vector2 &ballPlacementTarget);

        static double setNumTreePosP();
        static void setLuthPosP(double luthP);
        static double getNumTreePosI();
        static void setNumTreePosI(double luthI);
        static double getNumTreePosD();
        static void setNumTreePosD(double luthD);

        static double getNumTreeVelP();
        static void setNumTreeVelP(double luthP);
        static double getNumTreeVelI();
        static void setNumTreeVelI(double luthI);
        static double getNumTreeVelD();
        static void setNumTreeVelD(double luthD);

        static void sendHaltCommand();
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
