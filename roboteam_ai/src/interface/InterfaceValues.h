//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_INTERFACEVALUES_H
#define ROBOTEAM_AI_INTERFACEVALUES_H

#include <mutex>
#include <roboteam_utils/Vector2.h>

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

        static bool usesRefereeCommands();
        static void setUseRefereeCommands(bool useRefereeCommands);
        static const rtt::Vector2 &getBallPlacementTarget();
        static void setBallPlacementTarget(const rtt::Vector2 &ballPlacementTarget);

        static double getNumTreePosP();
        static void setLuthPosP(double luthP);
        static double getNumTreePosI();
        static void setLuthPosI(double luthI);
        static double getNumTreePosD();
        static void setLuthPosD(double luthD);

        static double getNumTreeVelP();
        static void setLuthVelP(double luthP);
        static double getNumTreeVelI();
        static void setLuthVelI(double luthI);
        static double getNumTreeVelD();
        static void setLuthVelD(double luthD);

        static void sendHaltCommand();
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
