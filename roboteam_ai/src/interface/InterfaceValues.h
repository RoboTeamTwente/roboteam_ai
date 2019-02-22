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
        static rtt::Vector2 ballPlacementTarget;
        static bool useRefereeCommands;

    public:
        static bool usesRefereeCommands();
        static void setUseRefereeCommands(bool useRefereeCommands);
        static const rtt::Vector2 &getBallPlacementTarget();
        static void setBallPlacementTarget(const rtt::Vector2 &ballPlacementTarget);

        static double getLuthPosP();
        static void setLuthPosP(double luthP);
        static double getLuthPosI();
        static void setLuthPosI(double luthI);
        static double getLuthPosD();
        static void setLuthPosD(double luthD);

        static double getLuthVelP();
        static void setLuthVelP(double luthP);
        static double getLuthVelI();
        static void setLuthVelI(double luthI);
        static double getLuthVelD();
        static void setLuthVelD(double luthD);

        static void sendHaltCommand();

};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
