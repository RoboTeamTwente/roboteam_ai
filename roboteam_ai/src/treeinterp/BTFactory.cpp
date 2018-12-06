#include <utility>

#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, bt::BehaviorTree::Ptr> BTFactory::strategyRepo;
std::map<std::string, bt::Node::Ptr>BTFactory::tacticsRepo;
std::string BTFactory::currentTree;
bool BTFactory::initialized = false;

/// Returns the Behaviour Tree Factory Singleton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();

    for (const auto &tacticName : Switches::tacticJsonFileNames) {
        auto BB = std::make_shared<bt::Blackboard>(); //TODO maybe make the BB somewhere else that makes sense
        auto tempMap = interpreter.makeTactics(tacticName, BB);
        for (auto &it : tempMap) tacticsRepo[it.first] = it.second; // may break
    }

    for (const auto &strategyName : Switches::strategyJsonFileNames) {
        auto tempMap = interpreter.getTrees("strategies/" + strategyName);
        for (auto &it : tempMap) strategyRepo[it.first] = it.second; // may break
    }

    initialized = true;
}
bt::BehaviorTree::Ptr BTFactory::getTree(std::string treeName) {
    if (strategyRepo.find(treeName) != strategyRepo.end()) {
        return strategyRepo.find(treeName)->second;
    }
    ROS_ERROR("No Strategy by that name");
    return strategyRepo.end()->second;
}

std::string BTFactory::getCurrentTree() {
    return currentTree;
}

void BTFactory::setCurrentTree(const std::string & newTree) {

    // only change if it is a different tree
    if (newTree != BTFactory::currentTree) {
//        auto tree = BTFactory::getFactory().getTree(BTFactory::currentTree);
//        if (tree) { // terminate tree if needed
//            tree->GetRoot()->terminate(tree->GetRoot()->getStatus());
//        }

    for (auto tacticRobotsPair : robotDealer::RobotDealer::getClaimedRobots()) {
        robotDealer::RobotDealer::removeTactic(tacticRobotsPair.first);
    }

    BTFactory::currentTree = newTree;
    }
}

bool BTFactory::isInitialized() {
    return BTFactory::initialized;
}
























