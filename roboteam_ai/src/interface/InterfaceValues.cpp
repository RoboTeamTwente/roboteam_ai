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



double InterfaceValues::luthVelP = 0;
double InterfaceValues::luthVelI = 0;
double InterfaceValues::luthVelD = 0;

double InterfaceValues::luthPosP = 0;
double InterfaceValues::luthPosI = 0;
double InterfaceValues::luthPosD = 0;

rtt::Vector2 InterfaceValues::ballPlacementTarget = {0, 0}; // initialize on middle of the field
bool InterfaceValues::useRefereeCommands = false;


std::mutex InterfaceValues::PIDMutex;
std::mutex InterfaceValues::BallPlacementMutex;
std::mutex InterfaceValues::RefMutex;


double InterfaceValues::getLuthPosP() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosP;
}

void InterfaceValues::setLuthPosP(double luthPP) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosP = luthPP;
}

double InterfaceValues::getLuthPosI() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosI;
}

void InterfaceValues::setLuthPosI(double luthPI) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosI = luthPI;
}

double InterfaceValues::getLuthPosD() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthPosD;
}

void InterfaceValues::setLuthPosD(double LuthPD) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthPosD = LuthPD;
}

double InterfaceValues::getLuthVelP() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelP;
}

void InterfaceValues::setLuthVelP(double luthVP) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthVelP = luthVP;
}

double InterfaceValues::getLuthVelI() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelI;
}

void InterfaceValues::setLuthVelI(double luthVI) {
    std::lock_guard<std::mutex> lock(PIDMutex);
    InterfaceValues::luthVelI = luthVI;
}

double InterfaceValues::getLuthVelD() {
    std::lock_guard<std::mutex> lock(PIDMutex);
    return luthVelD;
}

void InterfaceValues::setLuthVelD(double LuthVD) {
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

void InterfaceValues::setUseRefereeCommands(bool useRefereeCommands){
    std::lock_guard<std::mutex> lock(RefMutex);
    InterfaceValues::useRefereeCommands = useRefereeCommands;
}


} // interface
} // ai
} // rtt