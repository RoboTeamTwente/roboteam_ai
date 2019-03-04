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
double InterfaceValues::luthPosP = 0;
double InterfaceValues::luthPosI = 0;
double InterfaceValues::luthPosD = 0;

double InterfaceValues::luthVelP = 0;
double InterfaceValues::luthVelI = 0;
double InterfaceValues::luthVelD = 0;

rtt::Vector2 InterfaceValues::ballPlacementTarget = {0, 0}; // initialize on middle of the field
bool InterfaceValues::useRefereeCommands = false;
bool InterfaceValues::showDebugValuesInTerminal = true;

std::mutex InterfaceValues::pidMutex;
std::mutex InterfaceValues::ballPlacementMutex;
std::mutex InterfaceValues::refMutex;
std::mutex InterfaceValues::showDebugMutex;

double InterfaceValues::setNumTreePosP() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthPosP;
}

void InterfaceValues::setLuthPosP(double luthPP) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthPosP = luthPP;
}

double InterfaceValues::getNumTreePosI() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthPosI;
}

void InterfaceValues::setNumTreePosI(double luthPI) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthPosI = luthPI;
}

double InterfaceValues::getNumTreePosD() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthPosD;
}

void InterfaceValues::setNumTreePosD(double LuthPD) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthPosD = LuthPD;
}

double InterfaceValues::getNumTreeVelP() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthVelP;
}

void InterfaceValues::setNumTreeVelP(double luthVP) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthVelP = luthVP;
}

double InterfaceValues::getNumTreeVelI() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthVelI;
}

void InterfaceValues::setNumTreeVelI(double luthVI) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthVelI = luthVI;
}

double InterfaceValues::getNumTreeVelD() {
    std::lock_guard<std::mutex> lock(pidMutex);
    return luthVelD;
}

void InterfaceValues::setNumTreeVelD(double LuthVD) {
    std::lock_guard<std::mutex> lock(pidMutex);
    InterfaceValues::luthVelD = LuthVD;
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

} // interface
} // ai
} // rtt