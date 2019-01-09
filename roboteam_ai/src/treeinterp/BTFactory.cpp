#include <utility>

#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, bt::BehaviorTree::Ptr> BTFactory::strategyRepo;
std::map<std::string, bt::Node::Ptr>BTFactory::tacticsRepo;
std::map<std::string, bt::BehaviorTree::Ptr>BTFactory::keeperRepo;
std::string BTFactory::currentTree = "NaN";
std::string BTFactory::keeperTree;
int BTFactory::keeperID;
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

    for (const auto &strategyNameKeeper : Switches::keeperJsonFiles) {
        auto tempMap = interpreter.getTrees("keeper/" + strategyNameKeeper);
        for (auto &it : tempMap) keeperRepo[it.first] = it.second; // may break
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

void BTFactory::setCurrentTree(const std::string &newTree) {

    if (newTree != BTFactory::currentTree) {

        if (BTFactory::currentTree == "NaN") {
            BTFactory::currentTree = newTree;
            return;
        }
        BTFactory::getFactory().getTree(currentTree)->terminate(bt::Node::Status::Success);

        BTFactory::currentTree = newTree;
    }
}

bool BTFactory::isInitialized() {
    return BTFactory::initialized;
}

void BTFactory::setKeeperTree(const std::string &keeperTree_) {

    keeperTree = keeperTree_;

}
void BTFactory::setKeeper(int newID) {
    BTFactory::keeperID = newID;
}
bt::BehaviorTree::Ptr BTFactory::getKeeperTree() {
    return keeperRepo[keeperTree];
}
























