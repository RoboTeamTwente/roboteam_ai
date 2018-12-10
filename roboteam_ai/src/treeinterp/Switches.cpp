//
// Created by baris on 15/11/18.
//

#include <roboteam_ai/src/skills/GoToPosLuTh.h>
#include "Switches.h"

/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */


std::vector<std::string> Switches::tacticJsonFileNames =
        {"victoryDanceTactic",
         "randomTactic",
         "GetBallTestTactic",
         "DanceTactic",
         "DanceTactic2",
         "SimpleTactic",
         "haltTactic"};

std::vector<std::string> Switches::strategyJsonFileNames =
        {"victoryDanceStrategy",
         "randomStrategy",
         "GetBallTestStrategy",
         "DanceStrategy",
         "SimpleStrategy",
         "haltStrategy"};

std::vector<std::string> Switches::keeperJsonFiles =
        {"emptyForNow"};

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
    else if (name == "Repeater") {
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
    else if (name == "Halt") {
        node = std::make_shared<rtt::ai::Halt>(name, properties);
    }
    else if (name == "Rotate") {
        node = std::make_shared<rtt::ai::Rotate>(name, properties);
    }
    else if (name == "GoToPosLuTh_OLD") {
        node = std::make_shared<rtt::ai::GoToPosLuTh_OLD>(name, properties);
    }
    else if (name == "GoToPosLuTh") {
        node = std::make_shared<rtt::ai::GoToPosLuTh>(name, properties);
    }
    else if (name == "Dribble") {
        node = std::make_shared<rtt::ai::Dribble>(name, properties);
    }
    else if (name == "RotateToAngle") {
        node = std::make_shared<rtt::ai::RotateToAngle>(name, properties);
    }
    else if (name == "HasBall") {
        node = std::make_shared<rtt::ai::HasBall>(name, properties);
    }
    else if (name == "Keeper"){
        node = std::make_shared<rtt::ai::Keeper>(name,properties);
    }
    else {
        ROS_ERROR("ERROR: Leaf not found!! using GoToPos..");
        node = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }

    return node;
}

/// If you made a tactic node for a new tactic this is where you add that
bt::Node::Ptr Switches::tacticSwitch(std::string name, bt::Blackboard::Ptr properties) {

    std::map<std::string, std::map<std::string, robotType>> tactics = {
            {"randomTactic", {
                    {"random1", robotType::random},
                    {"random2", robotType::random},
                    {"random3", robotType::random},
                    {"random4", robotType::random},
                    {"random5", robotType::random},
                    {"random6", robotType::random},
                    {"random7", robotType::random}
            }
            },
            {"haltTactic", {
                    {"halt0", robotType::random},
                    {"halt1", robotType::random},
                    {"halt2", robotType::random},
                    {"halt3", robotType::random},
                    {"halt4", robotType::random},
                    {"halt5", robotType::random},
                    {"halt6", robotType::random},
                    {"halt7", robotType::random}
            }
            },
            {"GetBallTestTactic", {
                    {"FAKOFF", robotType::random}
            }
            },
            {"DanceTactic2", {
                    {"retarded", robotType::random},
                    {"Vright", robotType::random}
            }
            },
            {"DanceTactic", {
                    {"right", robotType::random},
                    {"letf", robotType::random}
            }
            },
            {"SimpleTactic", {
                    {"simpleStupidRobot", robotType::random}
            }
            }
    };

    bt::Node::Ptr node;

    if (name == "VerySpecialTacticThatWouldRequireSpecialClass") {
        node = std::make_shared<bt::VictoryDanceTactic>("VerySpecialTacticThatWouldRequireSpecialClass", properties);
    }
    else if (tactics.find(name) != tactics.end()) {
        node = std::make_shared<bt::DefaultTactic>(name, properties, tactics[name]);
    }
    else if (name == "victoryDanceTactic") {
        node = std::make_shared<bt::VictoryDanceTactic>("victoryDanceTactic", properties);
    }
    else if (name == "randomTactic") {
        node = std::make_shared<bt::RandomTactic>("randomTactic", properties);
    }
    return node;
}
