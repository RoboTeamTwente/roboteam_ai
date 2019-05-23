//
// Created by mrlukasbos on 18-1-19.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "Output.h"

namespace rtt {
namespace ai {
namespace interface {

// these values need to be set AFTER ros::init, so they are initialized with values in the constructor of mainwindow
pidVals Output::numTreePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::forcePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::basicPID = pidVals(0.0, 0.0, 0.0);


rtt::Vector2 Output::markerPosition = {0, 0}; // initialize on middle of the field
bool Output::useRefereeCommands = false;
bool Output::showDebugValuesInTerminal = true;
bool Output::timeOutAtTop = Constants::STD_TIMEOUT_TO_TOP();

std::mutex Output::pidMutex;
std::mutex Output::markerMutex;
std::mutex Output::refMutex;
std::mutex Output::showDebugMutex;

GameState Output::interfaceGameState("halt_strategy", "keeper_halt_tactic", "default");

void Output::sendHaltCommand() {
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

const Vector2& Output::getInterfaceMarkerPosition() {
    std::lock_guard<std::mutex> lock(markerMutex);
    return markerPosition;
}

void Output::setMarkerPosition(const Vector2 &ballPlacementTarget) {
    std::lock_guard<std::mutex> lock(markerMutex);
    Output::markerPosition = ballPlacementTarget;
}

bool Output::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(refMutex);
    return useRefereeCommands;
}

void Output::setUseRefereeCommands(bool _useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    Output::useRefereeCommands = _useRefereeCommands;
}

void Output::setShowDebugValues(bool showDebug) {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    Output::showDebugValuesInTerminal = showDebug;
}

bool Output::getShowDebugValues() {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    return Output::showDebugValuesInTerminal;
}

bool Output::showDebugTickTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_TICK_TIME_TAKEN();
}

bool Output::showDebugLongestTick() {
    return getShowDebugValues() && Constants::SHOW_LONGEST_TICK();
}

bool Output::showDebugNumTreeTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_TIME_TAKEN();
}

bool Output::showCoachTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_COACH_TIME_TAKEN();
}

bool Output::showDebugNumTreeInfo() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_DEBUG_INFO();
}

bool Output::showFullDebugNumTreeInfo() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_DEBUG_INFO() && Constants::SHOW_FULL_NUMTREE_DEBUG_INFO();
}

const pidVals &Output::getNumTreePid() {
    return numTreePID;
}

void Output::setNumTreePid(const pidVals &numTreePid) {
    numTreePID = numTreePid;
}

const pidVals &Output::getForcePid() {
    return forcePID;
}

void Output::setForcePid(const pidVals &forcePid) {
    forcePID = forcePid;
}

const pidVals &Output::getBasicPid() {
    return basicPID;
}

void Output::setBasicPid(const pidVals &basicPid) {
    basicPID = basicPid;
}

void Output::setTimeOutTop(bool top) {
    timeOutAtTop = top;
}

bool Output::isTimeOutAtTop() {
    return timeOutAtTop;
}

void Output::setKeeperTree(std::string name) {
    Output::interfaceGameState.keeperStrategyName = std::move(name);
}

void Output::setStrategyTree(std::string name) {
    Output::interfaceGameState.strategyName = std::move(name);
    Output::interfaceGameState.ballPositionAtStartOfGameState = world::world->getBall()->pos;
}

void Output::setRuleSetName(std::string name) {
    Output::interfaceGameState.ruleSetName = std::move(name);
}


void Output::setKeeperId(int id) {
    Output::interfaceGameState.keeperId = id;
}

const GameState &Output::getInterfaceGameState() {
    return Output::interfaceGameState;
}

void Output::setInterfaceGameState(const GameState &interfaceGameState) {
    Output::interfaceGameState = interfaceGameState;
}


} // interface
} // ai
} // rtt