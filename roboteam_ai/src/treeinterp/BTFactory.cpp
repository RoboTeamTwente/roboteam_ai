#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, std::map<std::string, bt::BehaviorTree>> BTFactory::strategyRepo;
std::vector<std::string> BTFactory::strategyNames;

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

    // Update the projectNames Vector with all the projectFiles we want.
    initialProjectNames();

    // Updates all the projects in the projectNames vector and adds them to strategyRepo
    for (const std::string &projectName : strategyNames) {
        updateProject(projectName);
    }
}

//TODO: add any trees you wish to load initially in the jsons folder here!!!
/// Inserts the initial projectNames into the vector
void BTFactory::initialProjectNames() {
    std::string initialNames[] = {
            "bigjson",
            "sample"
    };
    strategyNames.insert(strategyNames.end(), std::begin(initialNames), std::end(initialNames));
}






















