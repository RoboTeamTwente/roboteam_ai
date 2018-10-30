
//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"
#include "../skills/GoToPos.h"

/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Returns a BehaviorTree from a given name TESTED
std::map<std::string, bt::BehaviorTree> TreeInterpreter::getProject(std::string name) {
    std::map<std::string, bt::BehaviorTree> result;

    // Read a project from file
    auto project = jsonReader.readJSON(std::move(name));

    // Loop over all the trees in the project JSON and put them in the map
    // TODO: fix the names for the trees
    for (const json &tree : project["data"]["trees"]) {
        std::string treeID = tree["id"];
        bt::BehaviorTree currentTree = buildTreeFromJSON(tree);
        //std::cout << "Build tree with id: " << treeID<<std::endl;
        result.insert(std::pair<std::string, bt::BehaviorTree>(treeID, currentTree));
    }
    return result;
}

/// Returns one BehaviorTree in a project
bt::BehaviorTree TreeInterpreter::getTreeWithID(std::string projectName, std::string ID) {

    // Read a project from file
    auto project = jsonReader.readJSON(std::move(projectName));

    auto trees = project["data"]["trees"];
    for (json tree : trees) {
//      jsonReader.printJson(tree);
        //  std::cout << tree["title"] << std::endl;
        if (tree["id"] == ID) {
            // jsonReader.printJson(tree);
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

        // Loop and add all of the trees to the vector
        for (const json &current : trees) {
            result.push_back(current);
        }
    }
    else {
        std::cerr << "The JSON tree is not a project!" << std::endl;
    }

    return result;
}

/// Build a BehaviorTree from a JSON object
bt::BehaviorTree TreeInterpreter::buildTreeFromJSON(json jsonTree) {

    // ID of the root
    std::string rootID = jsonTree["root"];


    // Build the tree from the root
    bt::BehaviorTree behaviorTree;
    bt::Blackboard::Ptr globalBB = std::make_shared<bt::Blackboard>();
    behaviorTree.setProperties(globalBB);

    auto rootNode = TreeInterpreter::buildNode(jsonTree["nodes"][rootID], jsonTree, globalBB);
    behaviorTree.SetRoot(rootNode);
    return behaviorTree;
}

/// Builds nodes recursively from json objects
bt::Node::Ptr TreeInterpreter::buildNode(json nodeJSON, json tree, bt::Blackboard::Ptr globalBlackBoard) {

    // See if it is leaf

    // Then:
    //  Make a leaf out of it and then return it

    // Else:
    //  Create a Node(?) object
    //  Call addChild on that node with a recursive call to this function
    //  and for every child it has
    //  Ex: node->addChild(buildNode(nodeJSON["<aChild>"]))
    //  Return this Node

    //jsonReader.printJson(nodeJSON);
    if (TreeInterpreter::isLeaf(nodeJSON)) {
        // TODO: make leaf and return it
        // TODO put properties
        // ?? WTF types and classes are impossible
        // Copy Pasta example: bt::Leaf::Ptr counterA = std::make_shared<Counter>("A", 2);
        bt::Leaf::Ptr leaf;
        leaf = TreeInterpreter::makeLeafNode(nodeJSON["title"]);
        leaf->globalBB = globalBlackBoard;
        // leaf->setFiled("string") TODO
        return leaf;
    }

    // Not a leaf
    auto node = makeNonLeafNode(nodeJSON["name"]);
    node->globalBB = globalBlackBoard;

    // has only one child
    if (jsonReader.checkIfKeyExists("child", nodeJSON)) {
        // Find the child node in the json
        std::string childID = nodeJSON["child"];
        auto child = tree["nodes"][childID];
        // recursive call

        node->AddChild(TreeInterpreter::buildNode(child, tree, globalBlackBoard));
        return node;
    }
    // has multiple children
    for (std::string currentChildID : nodeJSON["children"]) {
        auto currentChild = tree["nodes"][currentChildID];
        // recursive call
        node->AddChild(TreeInterpreter::buildNode(currentChild, tree, globalBlackBoard));
    }
    return node;
}

bt::Node::Ptr TreeInterpreter::makeNonLeafNode(std::string name) {

    // C++ doesnt like switches with strings :(

    bt::Node::Ptr node;

    // TODO: check the namings from the bt module and fix/add them here

    // Some of the naming is archaic, but we need it here.
    if (name == "MemSelector" || name == "Selector" || name == "Priority" || name == "MemPriority") {
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
    else if (name == "Sequence" || name == "ParallelTactic" || name == "ParallelSequence" ||
            name == "ParallelTactic") {
        node = std::make_shared<bt::Sequence>();
    }
    else if (name == "Failer") {
        node = std::make_shared<bt::Failer>();
    }
    else if (name == "Inverter") {
        node = std::make_shared<bt::Inverter>();
    }
    else if (name == "Repeat") {
        node = std::make_shared<bt::Repeater>();
    }
    else if (name == "Succeeder") {
        node = std::make_shared<bt::Succeeder>();
    }
    else if (name == "UntilFail") {
        node = std::make_shared<bt::UntilFail>();
    }
    else if (name == "UntilSuccess" || name == "RepeatUntilSuccess") {
        node = std::make_shared<bt::UntilSuccess>();
    }
    else {
        std::cerr << "Node name with: " + name << std::endl;
    }
    return node;

}

/// Returns if there is any element in a json called "child" or "children"
bool TreeInterpreter::isLeaf(json jsonTree) {
    bool hasChild = jsonReader.checkIfKeyExists("child", jsonTree);
    bool hasChildren = jsonReader.checkIfKeyExists("children", jsonTree);
    return ! (hasChild || hasChildren);
}

/// Make a leaf node depending on the name of the node
bt::Leaf::Ptr TreeInterpreter::makeLeafNode(std::string name) {
    rtt::ai::Skill::Ptr skill;

    if (name == "Dummy") {
        // TODO: after importing the leaf subclasses make a switch here
        // leaf = make_some_condition_or_something()
    } else if (true) {
        bt::Blackboard::Ptr properties; // TODO parse properties
        return std::make_shared<rtt::ai::GoToPos>("goToPos", properties);
    }

    return skill;
}






