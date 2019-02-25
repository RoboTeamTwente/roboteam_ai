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

std::mutex InterfaceValues::PIDMutex;
std::mutex InterfaceValues::BallPlacementMutex;
std::mutex InterfaceValues::RefMutex;
std::mutex InterfaceValues::ShowDebugMutex;

double InterfaceValues::setNumTreePosP() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosP;
}

void InterfaceValues::setLuthPosP(double luthPP) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosP = luthPP;
}

double InterfaceValues::getNumTreePosI() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosI;
}

void InterfaceValues::setNumTreePosI(double luthPI) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosI = luthPI;
}

double InterfaceValues::getNumTreePosD() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosD;
}

void InterfaceValues::setNumTreePosD(double LuthPD) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosD = LuthPD;
}

double InterfaceValues::getNumTreeVelP() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelP;
}

void InterfaceValues::setNumTreeVelP(double luthVP) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthVelP = luthVP;
}

double InterfaceValues::getNumTreeVelI() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelI;
}

void InterfaceValues::setNumTreeVelI(double luthVI) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthVelI = luthVI;
}

double InterfaceValues::getNumTreeVelD() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelD;
}

void InterfaceValues::setNumTreeVelD(double LuthVD) {
    std::lock_guard<std::mutex> lock(PIDMutex);
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

void InterfaceValues::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(RefMutex);
    InterfaceValues::useRefereeCommands = useRefereeCommands;
}

void InterfaceValues::setShowDebugValues(bool showDebug) {
    std::lock_guard<std::mutex> lock(ShowDebugMutex);
    InterfaceValues::showDebugValuesInTerminal = showDebug;
}

bool InterfaceValues::getShowDebugValues() {
    std::lock_guard<std::mutex> lock(ShowDebugMutex);
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