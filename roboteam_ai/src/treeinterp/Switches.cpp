//
// Created by baris on 15/11/18.
//

#include "Switches.h"


/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */


std::vector<std::string> Switches::tacticJsonFileNames = {"testTactic", "testParallelTactic", "victoryDanceTactic","grsimTestTactic"};

std::vector<std::string> Switches::strategyJsonFileNames = {"testStrategy", "testParallelSequence", "victoryDanceStrategy","grsimTest"};



/// If you are touching this either you know what you are doing or you are making a mistake,
/// have a look around with the names and see if what you made is on the same level as these are
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

/// If you made a skill or a condition this is where you put them to use
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
    else if (name == "GoToPosLuTh") {
        node = std::make_shared<rtt::ai::GoToPosLuTh>(name, properties);
    }
    else if (name == "RotateToAngle"){
        node = std::make_shared<rtt::ai::RotateToAngle>(name,properties);
    }
    else {
        ROS_ERROR("ERROR: Leaf not found!! using GoToPos..");
        node = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }

    return node;
}

/// If you made a tactic node for a new tactic this is where you add that
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
    else if (name =="grsimTestTactic") {
        node= std::make_shared<bt::grsimTestTactic>("grsimTestTactic",properties);   }
    return node;
}
