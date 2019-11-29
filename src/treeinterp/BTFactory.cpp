//
// Created by baris on 04/10/18.
//

#include "treeinterp/BTFactory.h"
#include "treeinterp/TreeProtoType.h"


std::map<std::string, bt::BehaviorTree::Ptr> BTFactory::strategyRepo;
std::map<std::string, bt::Node::Ptr>BTFactory::tacticsRepo;
std::map<std::string, bt::BehaviorTree::Ptr>BTFactory::keeperRepo;
std::map<std::string, bt::BehaviorTree::Ptr>BTFactory::codeTrees;

std::string BTFactory::currentTree = "NaN";
std::string BTFactory::keeperTree;
std::mutex BTFactory::keeperTreeMutex;

// C++ code trees



bool BTFactory::weMadeTrees = false;

/// Initiate the BTFactory
void BTFactory::makeTrees() {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    BTFactory::weMadeTrees = false;

    std::cout << "Re-Make Trees From Json" << std::endl;

    // We need this instance of the TreeProtoType Class to use its methods. An alternative would be to make the methods static,
    // but this essentially makes them global which is dangerous and unnecessary in my opinion. Additionally, some settings
    // are initialized in the default constructor of TreeProtoType
    bt::TreeProtoType protoTypeTree;
    std::shared_ptr<bt::BehaviorTree> testingTree;
    testingTree = protoTypeTree.createOffensiveStrategy();

    /*
     * Here we store the C++ trees in a map, key = treename, val = cpp tree.
     * In order to do this in a cleaner way, maybe build trees automatically by going through directory
     */
    codeTrees["attackertree"] = protoTypeTree.createOffensiveStrategy();


    // TODO Remove this legacy code
    // If you think calling this over and over again is bad or slow you are partially correct. But if you optimize with
    //-O1 flag this takes like 20 ms so it is totally fine.
    TreeInterpreter interpreter = TreeInterpreter::getInstance();

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

    BTFactory::weMadeTrees = true;
    std::cout << "Done making trees" << std::endl;
}

/**
 * Currently we hijack this function and only return our current c++ tree with it.
 * @param treeName the name of the behaviour tree you are requesting.
 * @return The behaviourtree corresponding to that treename (currently hardcoded)
 */
bt::BehaviorTree::Ptr BTFactory::getTree(std::string treeName) {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);

    // Un-kill the code below by commenting the return statement to restore json functionality
    auto treefound = codeTrees.find("attackertree");
    return treefound->second;
    return codeTrees["attackertree"];

//    Leaving this code commented because it might be useful for later, depending on how we want to structure our tree storage
    if (strategyRepo.find(treeName) != strategyRepo.end()) {
        return strategyRepo.find(treeName)->second;
    }
    std::cerr << "NO STRATEGY BY THAT NAME:" << treeName.c_str() << std::endl;
    return nullptr;
}

std::string BTFactory::getCurrentTree() {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    return currentTree;
}

void BTFactory::setCurrentTree(const std::string &newTree) {
    {
        std::lock_guard<std::mutex> lock(keeperTreeMutex);

        if (newTree != BTFactory::currentTree) {

            if (BTFactory::currentTree == "NaN") {
                BTFactory::currentTree = newTree;
                return;
            }
        }
    }
    if (BTFactory::currentTree == "NaN" && BTFactory::getTree(currentTree)) {
        BTFactory::getTree(currentTree)->terminate(bt::Node::Status::Success);
    }

    rtt::ai::robotDealer::RobotDealer::halt();
    BTFactory::currentTree = newTree;

}

void BTFactory::setKeeperTree(const std::string &keeperTree_) {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    keeperTree = keeperTree_;
}

bt::BehaviorTree::Ptr BTFactory::getKeeperTree() {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    return keeperRepo[keeperTree];
}

std::string BTFactory::getKeeperTreeName() {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    return keeperTree;
}

void BTFactory::halt() {
    BTFactory::getTree(BTFactory::getCurrentTree())->terminate(bt::Node::Status::Success);
    BTFactory::setCurrentTree("NaN");
    rtt::ai::robotDealer::RobotDealer::halt();
}
bool BTFactory::hasMadeTrees() {
    std::lock_guard<std::mutex> lock(keeperTreeMutex);
    return BTFactory::weMadeTrees;
}



