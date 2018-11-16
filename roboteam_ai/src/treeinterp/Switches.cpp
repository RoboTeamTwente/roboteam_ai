//
// Created by baris on 15/11/18.
//

#include "Switches.h"
#include "../bt/Node.hpp"
#include "../bt/tactics/DemoTactic.h"
#include "../bt/tactics/ParallelSequenceTest.h"
#include "../bt/tactics/VictoryDanceTactic.h"
#include "../skills/Rotate.h"

std::vector<std::string> Switches::tacticNames = {"testTactic", "testParallelTactic", "victoryDanceTactic"};

std::vector<std::string> Switches::strategyNames = {"testStrategy", "testParallelSequence", "victoryDanceStrategy"};



bt::Node::Ptr Switches::nonLeafSwitch(std::string name) {

    bt::Node::Ptr node;

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
    else if (name == "Sequence" || name == "ParallelTactic") { // TODO: parallel here?
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

bt::Node::Ptr Switches::leafSwitch(std::string name, bt::Blackboard::Ptr properties) {

    bt::Node::Ptr node;

    if (name == "GoToPos") {
        node = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }
    else if (name == "Kick") {
        node = std::make_shared<rtt::ai::Kick>(name, properties);
    }
    else if (name == "Rotate") {
        node = std::make_shared<rtt::ai::Rotate>(name, properties);
    }
    else {
        node = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }

    return node;
}

bt::Node::Ptr Switches::tacticSwitch(std::string name, bt::Blackboard::Ptr properties) {

    bt::Node::Ptr node;

    if (name == "DemoTactic") {
        node = std::make_shared<bt::DemoTactic>("DemoTactic", properties);
    }
    else if (name == "ParallelSequenceTactic") {
        node = std::make_shared<bt::ParallelSequenceTactic>("ParallelSequenceTactic", properties);
    }
    else if (name == "VictoryDanceTactic") {
        node = std::make_shared<bt::VictoryDanceTactic>("VictoryDanceTactic", properties);
    }

    return node;
}
