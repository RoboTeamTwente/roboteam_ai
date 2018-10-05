#include <utility>

//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

/// Return a map of tree names and trees that belong to one project
std::map<std::string, bt::BehaviorTree> BTFactory::getProject(std::string projectName) {
    return std::map<std::string, bt::BehaviorTree>();
}

/// Update an entire project
void BTFactory::updateProject(std::string projectName) {

    auto project = interpreter.getProject(projectName);
    BTFactory::treeRepo[projectName] = project;

}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {
    auto tree = interpreter.getTree(projectName, treeName);

    BTFactory::treeRepo[projectName][treeName] = tree;


}

/// Returns the Behaviour Tree Factory Singelton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

std::map<std::string, std::map<std::string, bt::BehaviorTree>> getTreeRepo() {
    return BTFactory::treeRepo;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();
    // TODO: turn all of the jsons into BTs

}
