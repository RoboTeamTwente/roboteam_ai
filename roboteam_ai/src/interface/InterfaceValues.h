//
// Created by mrlukasbos on 18-1-19.
//

#ifndef ROBOTEAM_AI_INTERFACEVALUES_H
#define ROBOTEAM_AI_INTERFACEVALUES_H

#include <mutex>

namespace rtt {
namespace ai {
namespace interface {

class InterfaceValues {
private:
    static double luthP;
    static double luthI;
    static double luthD;
    static std::mutex PIDMutex;
    static std::mutex BallPlacementMutex;
    static rtt::Vector2 ballPlacementTarget;
public:
    static const Vector2& getBallPlacementTarget();
    static void setBallPlacementTarget(const Vector2& ballPlacementTarget);
    static double getLuthP();
    static void setLuthP(double luthP);
    static double getLuthI();
    static void setLuthI(double luthI);
    static double getLuthD();
    static void setLuthD(double luthD);
    static void sendHaltCommand();
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
