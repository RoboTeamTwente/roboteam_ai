//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, bt::BehaviorTree::Ptr> BTFactory::strategyRepo;
std::map<std::string, bt::Node::Ptr>BTFactory::tacticsRepo;
std::map<std::string, bt::BehaviorTree::Ptr>BTFactory::keeperRepo;
std::string BTFactory::currentTree = "NaN";
std::string BTFactory::keeperTree;


/// Initiate the BTFactory
void BTFactory::makeTrees() {
    std::cout << "Re-Make Trees From Json" << std::endl;

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
}

bt::BehaviorTree::Ptr BTFactory::getTree(std::string treeName) {
    if (strategyRepo.find(treeName) != strategyRepo.end()) {
        return strategyRepo.find(treeName)->second;
    }
    ROS_ERROR("NO STRATEGY BY THAT NAME:    %s\n\n\n", treeName.c_str());
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
        BTFactory::getTree(currentTree)->terminate(bt::Node::Status::Success);
        robotDealer::RobotDealer::halt();
        BTFactory::currentTree = newTree;
    }
}

void BTFactory::setKeeperTree(const std::string &keeperTree_) {
    keeperTree = keeperTree_;
}

bt::BehaviorTree::Ptr BTFactory::getKeeperTree() {
    return keeperRepo[keeperTree];
}

void BTFactory::halt() {
    BTFactory::getTree(BTFactory::getCurrentTree())->terminate(bt::Node::Status::Success);
    BTFactory::currentTree = "NaN";
    robotDealer::RobotDealer::halt();


}
























