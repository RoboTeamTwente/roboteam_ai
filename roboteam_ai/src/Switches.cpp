//
// Created by baris on 15/11/18.
//

#include "Switches.h"
//  ______________________
//  |                    |
//  |   INCLUDE TACTICS  |
//  |____________________|
//

#include "roboteam_ai/src/bt/tactics/DefaultTactic.h"

//  ______________________
//  |                    |
//  |   INCLUDE SKILLS   |
//  |____________________|
//

#include <roboteam_ai/src/skills/BallPlacementWithInterface.h>


#include <roboteam_ai/src/bt/RoleDivider.h>

#include <roboteam_ai/src/bt/composites/MemSelector.hpp>
#include <roboteam_ai/src/bt/composites/MemSequence.hpp>
#include <roboteam_ai/src/bt/composites/MemParallelSequence.h>
#include <roboteam_ai/src/bt/decorators/Failer.hpp>
#include <roboteam_ai/src/bt/decorators/UntilFail.hpp>
#include <roboteam_ai/src/bt/decorators/UntilSuccess.hpp>
#include <roboteam_ai/src/bt/composites/Selector.hpp>
#include <roboteam_ai/src/bt/composites/ParallelSequence.hpp>
#include <roboteam_ai/src/bt/decorators/Succeeder.hpp>
#include <roboteam_ai/src/bt/decorators/Repeater.hpp>
#include <roboteam_ai/src/bt/decorators/Inverter.hpp>
#include <roboteam_ai/src/bt/composites/Sequence.hpp>

/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */

using robotType = rtt::ai::robotDealer::RobotType;

std::vector<std::string> Switches::tacticJsonFileNames = {
        "ball_placement_interface_tactic"
};

std::vector<std::string> Switches::strategyJsonFileNames = {
        "interface_ball_placement_strategy"
};

std::vector<std::string> Switches::keeperJsonFiles = {
        };

/// If you are touching this either you know what you are doing or you are making a mistake,
/// have a look around with the names and see if what you made is on the same level as these are
bt::Node::Ptr Switches::nonLeafSwitch(std::string name) {
    std::map<std::string, bt::Node::Ptr> map;

    map["MemSelector"] = std::make_shared<bt::MemSelector>();
    map["MemSequence"] = std::make_shared<bt::MemSequence>();
    map["ParallelSequence"] = std::make_shared<bt::ParallelSequence>();
    map["MemParallelSequence"] = std::make_shared<bt::MemParallelSequence>();
    map["Selector"] = std::make_shared<bt::Selector>();
    map["Sequence"] = std::make_shared<bt::Sequence>();
    map["Inverter"] = std::make_shared<bt::Inverter>();
    map["Failer"] = std::make_shared<bt::Failer>();
    map["Repeat"] = std::make_shared<bt::Repeater>();
    map["Repeater"] = std::make_shared<bt::Repeater>();
    map["Succeeder"] = std::make_shared<bt::Succeeder>();
    map["UntilFail"] = std::make_shared<bt::UntilFail>();
    map["UntilSuccess"] = std::make_shared<bt::UntilSuccess>();
    map["RoleDivider"] = std::make_shared<bt::RoleDivider>();

    if (map.find(name) != map.end()) {
        return map[name];
    }
    else {

        ROS_ERROR("Faulty Control Node! Never should happen!");
        return bt::Node::Ptr();
    }

}

/// If you made a skill or a condition this is where you put them to use
bt::Node::Ptr Switches::leafSwitch(std::string name, bt::Blackboard::Ptr properties) {
    std::map<std::string, bt::Node::Ptr> map;

    map["BallPlacementWithInterface"] = std::make_shared<rtt::ai::BallPlacementWithInterface>(name, properties);

    if (map.find(name) != map.end()) {
        return map[name];
    }
    else {

        std::cerr << "\nTHE LEAF IS NOT REGISTERED IN SWITCHES: "<< name << std::endl;
        return bt::Node::Ptr();
    }
}

/// If you made a tactic node for a new tactic this is where you add that
bt::Node::Ptr Switches::tacticSwitch(std::string name, bt::Blackboard::Ptr properties) {

    // The second one is not a map because we want to keep the order

    std::map<std::string, std::vector<std::pair<std::string, robotType>>> tactics = {

            {"ball_placement_interface_tactic", {
                    {"follow_interface", robotType::WORKING_DRIBBLER},
            }
            }
    };

    bt::Node::Ptr node;
    if (tactics.find(name) != tactics.end()) {
        node = std::make_shared<bt::DefaultTactic>(name, properties, tactics[name]);
    }
    else {
        ROS_ERROR("\n\n\nTHE TACTIC DOES NOT HAVE ROBOTS SPECIFIED IN THE SWITCHES:    %s\n\n\n", name.c_str());
    }
    return node;
}

void Switches::runErrorHandler(std::map<std::string, std::map<std::string, robotType>> tactics) {

    for (auto &item : tactics) { // <--- NOT A CONST REFERENCE WOW MAN MAN MAN  -Team (int)Twee(nte)
        if (std::find(tacticJsonFileNames.begin(), tacticJsonFileNames.end(), item.first)
                == tacticJsonFileNames.end()) {
            ROS_ERROR("THE FOLLOWING TACTIC IS MISSING THE FILE:   %s\n\n\n", item.first.c_str());
        }
    }

}
