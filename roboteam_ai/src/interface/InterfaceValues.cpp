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
double InterfaceValues::numTreePosP = 0;
double InterfaceValues::numTreePosI = 0;
double InterfaceValues::numTreePosD = 0;

rtt::Vector2 InterfaceValues::ballPlacementTarget = {0, 0}; // initialize on middle of the field
bool InterfaceValues::useRefereeCommands = false;
bool InterfaceValues::showDebugValuesInTerminal = true;

std::mutex InterfaceValues::pidMutex;
std::mutex InterfaceValues::ballPlacementMutex;
std::mutex InterfaceValues::refMutex;
std::mutex InterfaceValues::showDebugMutex;

double InterfaceValues::getNumTreePosP() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return numTreePosP;
}

void InterfaceValues::setNumTreePosP(double numTreePP) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::numTreePosP = numTreePP;
}

double InterfaceValues::getNumTreePosI() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return numTreePosI;
}

void InterfaceValues::setNumTreePosI(double numTreePI) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::numTreePosI = numTreePI;
}

double InterfaceValues::getNumTreePosD() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return numTreePosD;
}

void InterfaceValues::setNumTreePosD(double numTreePD) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::numTreePosD = numTreePD;
}

void InterfaceValues::sendHaltCommand() {
    rtt::ai::Pause pause;

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
    std::lock_guard<std::mutex> lock(ballPlacementMutex);
    return ballPlacementTarget;
}

void InterfaceValues::setBallPlacementTarget(const Vector2& ballPlacementTarget) {
    std::lock_guard<std::mutex> lock(ballPlacementMutex);
    InterfaceValues::ballPlacementTarget = ballPlacementTarget;
}

bool InterfaceValues::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(refMutex);
    return useRefereeCommands;
}

void InterfaceValues::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    InterfaceValues::useRefereeCommands = useRefereeCommands;
}

void InterfaceValues::setShowDebugValues(bool showDebug) {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    InterfaceValues::showDebugValuesInTerminal = showDebug;
}

bool InterfaceValues::getShowDebugValues() {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    return InterfaceValues::showDebugValuesInTerminal;
}

bool InterfaceValues::showDebugTickTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_TICK_TIME_TAKEN();
}

bool InterfaceValues::showDebugLongestTick() {
    return getShowDebugValues() && Constants::SHOW_LONGEST_TICK();
}

bool InterfaceValues::showDebugNumTreeTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_TIME_TAKEN();
}

bool InterfaceValues::showDebugNumTreeInfo() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_DEBUG_INFO();
}

bool InterfaceValues::showFullDebugNumTreeInfo() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_DEBUG_INFO() && Constants::SHOW_FULL_NUMTREE_DEBUG_INFO();
}


} // interface
} // ai
} // rtt