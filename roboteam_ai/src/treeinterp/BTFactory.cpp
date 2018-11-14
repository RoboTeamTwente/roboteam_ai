#include <utility>

#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

static bool isInitiated = false;
std::map<std::string, bt::BehaviorTree::Ptr> BTFactory::strategyRepo;
std::map<std::string, bt::Node::Ptr>BTFactory::tacticsRepo;



/// Update an entire project
void BTFactory::updateProject(std::string projectName) {
    auto trees = interpreter.getTrees(std::move(projectName));
    for (auto tree : trees) {
        strategyRepo.find(tree.first)->second = tree.second;
    }
}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {
    auto trees = interpreter.getTrees(std::move(projectName));
    for (auto tree : trees) {
        if (tree.first == treeName) {
            strategyRepo.find(treeName)->second = tree.second;
        }
    }
}

/// Returns the Behaviour Tree Factory Singleton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();

    // Interpret all the tactics and put them in tactics repo as Node::Ptr
    // TODO: find a solution for BB passing
    auto BB = std::make_shared<bt::Blackboard>();
    // TODO: automate
    tacticsRepo = interpreter.makeTactics("testParallelTactic", BB);
    strategyRepo = interpreter.getTrees("strategies/testParallelSequence");



}
bt::BehaviorTree::Ptr BTFactory::getTree(std::string treeName) {
    if (strategyRepo.find(treeName) != strategyRepo.end()) {
        return strategyRepo.find(treeName)->second;
    }
    ROS_ERROR("No Strategy by that name");
    return strategyRepo.end()->second;
}

bool BTFactory::isIsInitiated() const {
    return isInitiated;
}

void BTFactory::setIsInitiated(bool newBool) {
    isInitiated = newBool;
}
























