//
// Created by mrlukasbos on 18-1-19.
//

#include "interface/api/Output.h"

#include <include/roboteam_ai/world/World.hpp>

namespace rtt::ai::interface {


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







void Output::setTimeOutTop(bool top) { timeOutAtTop = top; }

bool Output::isTimeOutAtTop() { return timeOutAtTop; }

void Output::setRuleSetName(std::string name) { Output::interfaceGameState.ruleSetName = std::move(name); }

void Output::setKeeperId(int id) { Output::interfaceGameState.keeperId = id; }

const GameState &Output::getInterfaceGameState() { return Output::interfaceGameState; }

void Output::setInterfaceGameState(GameState interfaceGameState) {
    // keep the keeper the same
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}




}  // namespace rtt::ai::interface