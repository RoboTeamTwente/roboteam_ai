#include <utility>

//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"
#include <unistd.h>



/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Build a BehaviorTree from a json object
bt::BehaviorTree TreeInterpreter::buildTreeFromJson(json json) {
    return bt::BehaviorTree();
}

/// Returns a BehaviorTree from a given name
bt::BehaviorTree TreeInterpreter::getTree(std::string name) {

    bt::BehaviorTree resultTree = buildTreeFromJson(readJson(std::move(name)));

    return resultTree;
}

/// Read JSON from a file
json TreeInterpreter::readJson(std::string fileName) {
    char * dir = getcwd(nullptr, 0);
    std::cout << dir << std::endl;

    std::ifstream ifs("/home/baris/roboteamtwente/workspace/src/roboteam_ai/roboteam_ai/src/treeinterp/jsons/" + fileName + ".json");
    json result = json::parse(ifs);


    return result;
}

