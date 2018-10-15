
//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"

/// Return a TreeInterpreter singleton
TreeInterpreter& TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Returns a BehaviorTree from a given name
std::map<std::string, bt::BehaviorTree> TreeInterpreter::getProject(std::string name) {
    std::map<std::string, bt::BehaviorTree> result;

    // Read a project from file
    auto project = jsonReader.readJSON(std::move(name));

    // Loop over all the trees in the project JSON and put them in the map
    // TODO: fix the names for the trees
    for (const json& tree : project["trees"]) {
        bt::BehaviorTree currentTree = buildTreeFromJSON(tree);
        result.insert(std::pair<std::string, bt::BehaviorTree>("temp_name", currentTree));
    }
    return result;
}

/// Returns one BehaviorTree in a project
bt::BehaviorTree TreeInterpreter::getTreeWithID(std::string projectName, std::string ID) {

    // Read a project from file
    auto project = jsonReader.readJSON(std::move(projectName));

    for (auto tree : project["trees"]) {
        if (tree["id"] == ID) {
            return buildTreeFromJSON(tree);
        }
    }
    // return
    std::cerr << "No Tree with that ID" << std::endl;
}
/// Parse from the project JSON small tree JSONs
std::vector<json> TreeInterpreter::parseSmallJSONs(json input) {

    std::vector<json> result;

    // First check if it is indeed a project
    if (input["data"]["scope"] == "project") {
        auto trees = input["data"]["trees"];

        // Loop and add all of the tress to the vector
        for (const json& current : trees) {
            result.push_back(current);
        }
    }
    else {
        std::cerr << "MURDER ME" << std::endl;
    }

    return result;
}

/// Build a BehaviorTree from a JSON object
bt::BehaviorTree TreeInterpreter::buildTreeFromJSON(json jsonTree) {

    // ID of the root
    std::string rootID = jsonTree["root"];
    auto rootNode = TreeInterpreter::buildNode(jsonTree[rootID]);

    // Build the tree from the root
    bt::BehaviorTree behaviorTree(rootNode);
    return behaviorTree;
}

/// Builds nodes recursively from json objects
bt::Node::Ptr TreeInterpreter::buildNode(json nodeJSON) {

    // See if it is leaf

    // Then:
    //  Make a leaf out of it and then return it

    // Else:
    //  Create a Node(?) object
    //  Call addChild on that node with a recursive call to this function
    //  and for every child it has
    //  Ex: node->addChild(buildNode(nodeJSON["<aChild>"]))
    //  Return this Node

    if (TreeInterpreter::isLeaf(nodeJSON)) {
        // TODO: make leaf and return it
        // ?? WTF types and classes are impossible
        // Copy Pasta example: bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
        return bt::Leaf::Ptr();
    }

    auto node = makeNonLeafNode(nodeJSON["name"]); // TODO: Fix

    // has only one child
    if (! nodeJSON["children"]) {
        // recursive call
        node->AddChild(TreeInterpreter::buildNode(nodeJSON["child"]));
        return node;
    }
    // has multiple children
    for (const auto& child : nodeJSON["children"]) {
        // recursive call
        node->AddChild(TreeInterpreter::buildNode(child));
    }
    return node;
}

bt::Node::Ptr TreeInterpreter::makeNonLeafNode(std::string name) {

    // C++ doesnt like switches with strings :(

    bt::Node::Ptr node;

    // TODO: check the namings from the bt module and fix/add them here

    if (name == "MemSelector") {
        node = std::make_shared<bt::MemSelector>();
    }
    else if (name == "MemSequence") {
        node = std::make_shared<bt::MemSequence>();
    }
    else if (name == "ParallelSequence") {
        node = std::make_shared<bt::ParallelSequence>();
    }
    else if (name == "Selector") {
        node = std::make_shared<bt::Selector>();
    }
    else if (name == "Sequence") {
        node = std::make_shared<bt::Sequence>();
    }
    else if (name == "Failer") {
        node = std::make_shared<bt::Failer>();
    }
    else if (name == "Inverter") {
        node = std::make_shared<bt::Inverter>();
    }
    else if (name == "Repeater") {
        node = std::make_shared<bt::Repeater>();
    }
    else if (name == "Succeeder") {
        node = std::make_shared<bt::Succeeder>();
    }
    else if (name == "UntilFail") {
        node = std::make_shared<bt::UntilFail>();
    }
    else if (name == "UntilSuccess") {
        node = std::make_shared<bt::UntilSuccess>();
    }
    else {
        std::cerr << "Node name with: " + name << std::endl;
    }
    return node;

}

bool TreeInterpreter::isLeaf(json jsonTree) {
    return ! (jsonTree["child"] || jsonTree["children"]);
}







