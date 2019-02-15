//
// Created by mrlukasbos on 18-1-19.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "InterfaceValues.h"

namespace rtt {
namespace ai {
namespace interface {

// these values need to be set AFTER ros::init, so they are initialized with values in the constructor of mainwindow
double InterfaceValues::luthP = 0;
double InterfaceValues::luthI = 0;
double InterfaceValues::luthD = 0;
rtt::Vector2 InterfaceValues::ballPlacementTarget = {0, 0}; // initialize on middle of the field
bool InterfaceValues::useRefereeCommands = false;


std::mutex InterfaceValues::PIDMutex;
std::mutex InterfaceValues::BallPlacementMutex;
std::mutex InterfaceValues::RefMutex;


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
    rtt::Pause pause = rtt::Pause();
    if (pause.getPause()) {
        // Already halted so unhalt
        pause.setPause(false);
    }
    else {
        pause.setPause(true);
        pause.haltRobots();
    }

}

const Vector2& InterfaceValues::getBallPlacementTarget() {
    std::lock_guard<std::mutex> lock(BallPlacementMutex);
    return ballPlacementTarget;
}

void InterfaceValues::setBallPlacementTarget(const Vector2& ballPlacementTarget) {
    std::lock_guard<std::mutex> lock(BallPlacementMutex);
    InterfaceValues::ballPlacementTarget = ballPlacementTarget;
}

bool InterfaceValues::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(RefMutex);
    return useRefereeCommands;
}

void InterfaceValues::setUseRefereeCommands(bool useRefereeCommands){
    std::lock_guard<std::mutex> lock(RefMutex);
    InterfaceValues::useRefereeCommands = useRefereeCommands;
}

} // interface
} // ai
} // rtt