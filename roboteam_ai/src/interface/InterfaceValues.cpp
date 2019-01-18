//
// Created by mrlukasbos on 18-1-19.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include "InterfaceValues.h"

namespace rtt {
namespace ai {
namespace interface {

double InterfaceValues::luthP = constants::standard_luth_P;
double InterfaceValues::luthI = constants::standard_luth_I;
double InterfaceValues::luthD = constants::standard_luth_D;
std::mutex InterfaceValues::PIDMutex;

double InterfaceValues::getLuthP() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthP;
}

void InterfaceValues::setLuthP(double luthP) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthP = luthP;
}

double InterfaceValues::getLuthI() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthI;
}

void InterfaceValues::setLuthI(double luthI) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthI = luthI;
}

double InterfaceValues::getLuthD() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthD;
}

void InterfaceValues::setLuthD(double LuthD) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthD = LuthD;
}
void InterfaceValues::sendHaltCommand() {

// this function triggers when the halt button is clicked!
}

} // interface
} // ai
} // rtt