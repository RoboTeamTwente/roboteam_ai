//
// Created by mrlukasbos on 18-1-19.
//

#include "interface/api/Output.h"

#include <include/roboteam_ai/world/World.hpp>

namespace rtt::ai::interface {


bool Output::useRefereeCommands = false;

std::mutex Output::refMutex;

GameState Output::interfaceGameState("halt_strategy", "default");


bool Output::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(refMutex);
    return useRefereeCommands;
}

void Output::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    Output::useRefereeCommands = useRefereeCommands;
}

void Output::setRuleSetName(std::string name) { Output::interfaceGameState.ruleSetName = std::move(name); }

void Output::setKeeperId(int id) { Output::interfaceGameState.keeperId = id; }

const GameState &Output::getInterfaceGameState() { return Output::interfaceGameState; }

void Output::setInterfaceGameState(GameState interfaceGameState) {
    // keep the keeper the same
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}




}  // namespace rtt::ai::interface