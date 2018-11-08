#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, std::map<std::string, bt::BehaviorTree>> BTFactory::strategyRepo;

/// Return a map of tree names and trees that belong to one project
std::map<std::string, bt::BehaviorTree> BTFactory::getProject(std::string projectName) {
    return strategyRepo[projectName];
}

/// Update an entire project
void BTFactory::updateProject(std::string projectName) {
    auto project = interpreter.getProject(projectName);
    strategyRepo[projectName] = project;
}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {

    auto tree = interpreter.getTreeWithID(projectName, treeName);
    strategyRepo[projectName][treeName] = tree;

}

/// Returns the Behaviour Tree Factory Singleton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

std::map<std::string, std::map<std::string, bt::BehaviorTree>> BTFactory::getTreeRepo() {
    return strategyRepo;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();

    // Interpret all the tactics and put them in tactics repo as Node::Ptr
    // TODO loop the actual folder




}























