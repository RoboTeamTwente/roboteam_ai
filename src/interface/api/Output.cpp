//
// Created by mrlukasbos on 18-1-19.
//

#include "interface/api/Output.h"

#include "world/World.hpp"

namespace rtt::ai::interface {

// these values are default initialized here, but will be updated once mainWidget.cpp constructs the PID widget.
pidVals Output::numTreePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::receivePID = pidVals(0.0, 0.0, 0.0);
pidVals Output::interceptPID = pidVals(0.0, 0.0, 0.0);
pidVals Output::keeperPID = pidVals(0.0, 0.0, 0.0);
pidVals Output::keeperInterceptPID = pidVals(0.0, 0.0, 0.0);

rtt::Vector2 Output::markerPosition = {0, 0};  // initialize on middle of the field
bool Output::useRefereeCommands = false;
bool Output::timeOutAtTop = Constants::STD_TIMEOUT_TO_TOP();

std::mutex Output::markerMutex;
std::mutex Output::refMutex;

GameState Output::interfaceGameState("halt_strategy", "default");

void Output::sendHaltCommand() {
    rtt::ai::Pause pause;
    auto const &[_, world] = rtt::world::World::instance();
    // TODO: This check prevents a segfault when we don't have a world (roobthub_world is off), but it should be checked earlier I think
    if (world->getWorld().has_value()) {
        if (pause.getPause()) {
            // Already halted so unhalt
            pause.setPause(false);
        } else {
            pause.setPause(true);
            pause.haltRobots(world);
        }
    }

    else {
        RTT_WARNING("Cannot pause robots, there is no world! Check roboteam_world")
    }
}

const Vector2 &Output::getInterfaceMarkerPosition() {
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

void Output::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    Output::useRefereeCommands = useRefereeCommands;
}

bool Output::isTimeOutAtTop() { return timeOutAtTop; }

void Output::setRuleSetName(std::string name) { Output::interfaceGameState.ruleSetName = std::move(name); }

void Output::setKeeperId(int id) { Output::interfaceGameState.keeperId = id; }

void Output::setBallPlacerId(int id) {}

const GameState &Output::getInterfaceGameState() { return Output::interfaceGameState; }

void Output::setInterfaceGameState(GameState interfaceGameState) {
    // keep the keeper the same
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}

const pidVals &Output::getNumTreePid() { return numTreePID; }

void Output::setNumTreePid(const pidVals &numTreePid) { numTreePID = numTreePid; }

const pidVals &Output::getReceivePid() { return receivePID; }

void Output::setReceivePid(const pidVals &receivePid) { receivePID = receivePid; }

const pidVals &Output::getInterceptPid() { return interceptPID; }

void Output::setInterceptPid(const pidVals &interceptPid) { interceptPID = interceptPid; }

const pidVals &Output::getKeeperPid() { return keeperPID; }

void Output::setKeeperPid(const pidVals &keeperPid) { keeperPID = keeperPid; }

const pidVals &Output::getKeeperInterceptPid() { return keeperInterceptPID; }

void Output::setKeeperInterceptPid(const pidVals &keeperInterceptPid) { keeperInterceptPID = keeperInterceptPid; }

}  // namespace rtt::ai::interface