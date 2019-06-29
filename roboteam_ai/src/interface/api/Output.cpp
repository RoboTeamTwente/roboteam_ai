//
// Created by mrlukasbos on 18-1-19.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include "Output.h"
#include "../../world/Ball.h"

namespace rtt {
namespace ai {
namespace interface {

// these values need to be set AFTER ros::init, so they are initialized with values in the constructor of mainwindow
pidVals Output::numTreePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::basicPID = pidVals(0.0, 0.0, 0.0);
pidVals Output::keeperPID = pidVals(0.0, 0.0, 0.0);
pidVals Output::keeperInterceptPID = pidVals(0.0, 0.0, 0.0);
pidVals Output::ballHandlePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::shotControllerPID = pidVals(0.0, 0.0, 0.0);

rtt::Vector2 Output::markerPosition = {0, 0}; // initialize on middle of the field
bool Output::showDebugValuesInTerminal = true;
bool Output::timeOutAtTop = Constants::STD_TIMEOUT_TO_TOP();

std::mutex Output::markerMutex;
std::mutex Output::refMutex;
std::mutex Output::showDebugMutex;

const Vector2& Output::getInterfaceMarkerPosition() {
    std::lock_guard<std::mutex> lock(markerMutex);
    return markerPosition;
}

void Output::setMarkerPosition(const Vector2 &ballPlacementTarget) {
    std::lock_guard<std::mutex> lock(markerMutex);
    Output::markerPosition = ballPlacementTarget;
}

void Output::setShowDebugValues(bool showDebug) {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    Output::showDebugValuesInTerminal = showDebug;
}

bool Output::getShowDebugValues() {
    std::lock_guard<std::mutex> lock(showDebugMutex);
    return Output::showDebugValuesInTerminal;
}

bool Output::showDebugNumTreeTimeTaken() {
    return getShowDebugValues() && Constants::SHOW_NUMTREE_TIME_TAKEN();
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

const pidVals &Output::getShotControllerPID() {
    return shotControllerPID;
}

void Output::setShotControllerPID(const pidVals &shotControllerPID) {
    Output::shotControllerPID = shotControllerPID;
}

const pidVals &Output::getKeeperPid() {
    return keeperPID;
}

void Output::setKeeperPid(const pidVals &keeperPid) {
    keeperPID = keeperPid;
}

const pidVals &Output::getKeeperInterceptPid() {
    return keeperInterceptPID;
}

void Output::setKeeperInterceptPid(const pidVals &keeperInterceptPid) {
    keeperInterceptPID = keeperInterceptPid;
}

const pidVals &Output::getBallHandlePid() {
    return ballHandlePID;
}

void Output::setBallHandlePid(const pidVals &ballHandlePid) {
    ballHandlePID = ballHandlePid;
}


} // interface
} // ai
} // rtt