#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

///This is a list of all the projects we want to read in /roboteam_ai/src/treeinterp/jsons
static std::vector<std::string> projectNames;

/// This is where all the BTs are kept
static std::map<std::string, std::map<std::string, bt::BehaviorTree>> treeRepo;

/// Return a map of tree names and trees that belong to one project TODO
std::map<std::string, bt::BehaviorTree> BTFactory::getProject(std::string projectName) {
    return treeRepo[projectName];
}

/// Update an entire project
void BTFactory::updateProject(std::string projectName) {

    auto project = interpreter.getProject(projectName);
    treeRepo[projectName] = project;

}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {

    auto tree = interpreter.getTreeWithID(projectName, treeName);
    treeRepo[projectName][treeName] = tree;


}

/// Returns the Behaviour Tree Factory Singleton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

std::map<std::string, std::map<std::string, bt::BehaviorTree>> BTFactory::getTreeRepo() {
    return treeRepo;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();

    // Update the projectNames Vector with all the projectFiles we want.
    initialProjectNames();

    // Updates all the projects in the projectNames vector and adds them to treeRepo
    for (const std::string &projectName : projectNames){
        updateProject(projectName);
    }

    // TODO: turn all of the jsons into BTs

}

/// Inserts the initial projectNames into the vector
void BTFactory::initialProjectNames() {
    std::string initialNames[] = {
            "bigjson",
            "sample"
            };
    projectNames.insert(projectNames.end(),std::begin(initialNames),std::end(initialNames));
}