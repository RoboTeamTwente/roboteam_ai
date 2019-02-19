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
    static double luthP;
    static double luthI;
    static double luthD;
    static QString haltText;
    static QString haltColor;
    static std::mutex PIDMutex;
    static std::mutex BallPlacementMutex;
    static std::mutex RefMutex;
    static rtt::Vector2 ballPlacementTarget;
    static bool useRefereeCommands;


    public:
    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2& getBallPlacementTarget();
    static void setBallPlacementTarget(const rtt::Vector2& ballPlacementTarget);
    static double getLuthP();
    static void setLuthP(double luthP);
    static double getLuthI();
    static void setLuthI(double luthI);
    static double getLuthD();
    static void setLuthD(double luthD);
    static void sendHaltCommand();
    static void setHaltText();
    static QString getHaltText();
    static QString getHaltColor();
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
