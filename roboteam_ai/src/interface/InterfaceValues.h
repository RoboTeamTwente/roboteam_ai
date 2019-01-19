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
public:
    static double getLuthP();
    static void setLuthP(double luthP);
    static double getLuthI();
    static void setLuthI(double luthI);
    static double getLuthD();
    static void setLuthD(double luthD);
};

}
}
}

#endif //ROBOTEAM_AI_INTERFACEVALUES_H
