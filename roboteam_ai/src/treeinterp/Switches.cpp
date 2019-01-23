//
// Created by baris on 15/11/18.
//

#include "Switches.h"


//  ______________________
//  |                    |
//  |   INCLUDE TACTICS  |
//  |____________________|
//

#include "../bt/tactics/VictoryDanceTactic.h"
#include "../bt/tactics/DefaultTactic.h"

//  ______________________
//  |                    |
//  |   INCLUDE SKILLS   |
//  |____________________|
//

#include "../skills/Chip.h"
#include "../skills/Dribble.h"
#include "../skills/GoToPosLuTh.h"
#include "roboteam_ai/src/skills/SkillGoToPos.h"
#include "../skills/Halt.h"
#include "../skills/Kick.h"
#include "../skills/Harass.h"
#include "../skills/RotateToAngle.h"
#include "../skills/GoToPos.h"
#include "../skills/Keeper.h"
#include "../skills/GetBall.h"
#include "../skills/Attack.h"
#include "../skills/Pass.h"
#include <roboteam_ai/src/skills/InterceptBall.h>
#include <roboteam_ai/src/skills/GoToPosLuTh.h>
#include <roboteam_ai/src/skills/InterceptBall.h>
#include "../skills/DribbleRotate.h"
#include <roboteam_ai/src/skills/Defend.h>
#include <roboteam_ai/src/skills/InterceptBall.h>
#include <roboteam_ai/src/skills/BasicGoToPos.h>

//  ______________________
//  |                    |
//  | INCLUDE CONDITIONS |
//  |____________________|
//

#include "../conditions/HasBall.hpp"
#include "../conditions/CanSeeGoal.h"
#include <roboteam_ai/src/conditions/TheyHaveBall.h>
#include <roboteam_ai/src/conditions/IsRobotClosestToBall.h>
#include <roboteam_ai/src/conditions/BallKickedToOurGoal.h>
#include <roboteam_ai/src/conditions/IsBallOnOurSide.h>
#include "../conditions/BallInDefenseAreaAndStill.h"
#include "../conditions/IsInDefenseArea.hpp"

/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */


std::vector<std::string> Switches::tacticJsonFileNames =
        {
         "QualificationTactic",
         "haltTactic",
         "OneAttackerTactic",
         "OneDefenderTactic",
         "SingleKeeperTactic",
         "TwoDefendersTactic",
         "OneAttackerOneDefenderTactic",
         "PassTactic",
         "Attactic",
         "KeeperTactic"};

std::vector<std::string> Switches::strategyJsonFileNames =
        {
         "QualificationStrategy",
         "haltStrategy",
         "PassStrategy",
         "DemoTeamTwenteStrategy",
         "twoPlayerStrategyV2",
         "threePlayerStrategyV2"
        };

std::vector<std::string> Switches::keeperJsonFiles =
        {};

/// If you are touching this either you know what you are doing or you are making a mistake,
/// have a look around with the names and see if what you made is on the same level as these are
bt::Node::Ptr Switches::nonLeafSwitch(std::string name) {
    std::map<std::string, bt::Node::Ptr> map;

    map["MemSelector"] =      std::make_shared<bt::MemSelector>();
    map["MemSequence"] =      std::make_shared<bt::MemSequence>();
    map["ParallelSequence"] = std::make_shared<bt::ParallelSequence>();
    map["Selector"] =         std::make_shared<bt::Selector>();
    map["Sequence"] =         std::make_shared<bt::Sequence>();
    map["Inverter"] =         std::make_shared<bt::Inverter>();
    map["Failer"] =           std::make_shared<bt::Failer>();
    map["Repeat"] =           std::make_shared<bt::Repeater>();
    map["Repeater"] =         std::make_shared<bt::Repeater>();
    map["Succeeder"] =        std::make_shared<bt::Succeeder>();
    map["UntilFail"] =        std::make_shared<bt::UntilFail>();
    map["UntilSuccess"] =     std::make_shared<bt::UntilSuccess>();

    if ( map.find(name) != map.end() ) {
        return map[name];
    }
    return bt::Node::Ptr();
}


/// If you made a skill or a condition this is where you put them to use
bt::Node::Ptr Switches::leafSwitch(std::string name, bt::Blackboard::Ptr properties) {
    std::map<std::string, bt::Node::Ptr> map;

    // skills (alphabetic order)

    /*
     * unused skills
     * chip
     * dribbleRotate
     * shootAtGoal
     * sideAttacker
     */

    map["Attack"] =                 std::make_shared<rtt::ai::Attack>(name, properties);
    map["Defend"] =                 std::make_shared<rtt::ai::Defend>(name, properties);
    map["DefendOnRobot"] =          std::make_shared<rtt::ai::DefendOnRobot>(name, properties);
    map["Dribble"] =                std::make_shared<rtt::ai::Dribble>(name, properties);
    map["GetBall"] =                std::make_shared<rtt::ai::GetBall>(name, properties);
    map["GoToPos"] =                std::make_shared<rtt::ai::GoToPos>(name, properties);
    map["GoToPosLuTh"] =            std::make_shared<rtt::ai::GoToPosLuTh>(name, properties);
    map["Halt"] =                   std::make_shared<rtt::ai::Halt>(name, properties);
    map["Harass"] =                 std::make_shared<rtt::ai::Harass>(name, properties);
    map["InterceptBall"] =          std::make_shared<rtt::ai::InterceptBall>(name, properties);
    map["Keeper"] =                 std::make_shared<rtt::ai::Keeper>(name, properties);
    map["Kick"] =                   std::make_shared<rtt::ai::Kick>(name, properties);
    map["Pass"] =                   std::make_shared<rtt::ai::Pass>(name, properties);
    map["RotateToAngle"] =          std::make_shared<rtt::ai::RotateToAngle>(name, properties);
    map["SkillGoToPos"] =           std::make_shared<rtt::ai::SkillGoToPos>(name, properties);
    map["BasicGoToPos"] =           std::make_shared<rtt::ai::BasicGoToPos>(name, properties);

    // conditions (alphabetic order)

    /*
     * unused Conditions
     * CanReachPoint
     * IsInDefenceArea
     * IsInZone
     * IsOnOurSide
     * WeHaveBall
     */

    map["BallKickedToOurGoal"] =    std::make_shared<rtt::ai::BallKickedToOurGoal>(name, properties);
    map["CanSeeGoal"] =             std::make_shared<rtt::ai::CanSeeGoal>(name, properties);
    map["HasBall"] =                std::make_shared<rtt::ai::HasBall>(name, properties);
    map["IsRobotClosestToBall"] =   std::make_shared<rtt::ai::IsRobotClosestToBall>(name, properties);
    map["TheyHaveBall"] =           std::make_shared<rtt::ai::TheyHaveBall>(name, properties);
    map["IsBallOnOurSide"] =        std::make_shared<rtt::ai::IsBallOnOurSide>(name, properties);
    map["BallInDefenseAreaAndStill"] = std::make_shared<rtt::ai::BallInDefenseAreaAndStill>(name, properties);
    map["IsInDefenseArea"] = std::make_shared<rtt::ai::IsInDefenseArea>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);

    if ( map.find(name) != map.end() ) {
        return map[name];
    }
    return bt::Node::Ptr();
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
            {"OneAttackerTactic", {
                    {"attacker", robotType::closeToTheirGoal}
            }
            },
            {"SingleKeeperTactic", {
                    {"keeper", robotType::closeToOurGoal}
            }
            },
            {"OneAttackerOneDefenderTactic", {
                    {"defender", robotType::closeToOurGoal},
                    {"attacker", robotType::closeToTheirGoal}
            }
            },
            {"OneDefenderTactic", {
                    {"defender", robotType::closeToTheirGoal}
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
            },
            {"TwoDefendersTactic", {
                    {"defender1", robotType::closeToOurGoal},
                    {"defender2", robotType::closeToOurGoal},
            }
            },

            {"SimpleDefendTactic", {
                    {"simpleDefender1", robotType::closeToOurGoal},
                    {"simpleDefender2", robotType::closeToOurGoal},
                    {"simpleDefender3", robotType::closeToOurGoal}
            }
            },
            {"SimpleDefendTactic_1", {
                    {"simpleDefender1", robotType::closeToOurGoal}
            }
            },
            {"Attactic", {
                    {"atak", robotType::random}
                    //{"atak", robotType::closeToBall},
            }
            },
            {"KeeperTacticV2", {
                    {"keeper", robotType::random}
            }
            },
            {"KeeperTactic", {
                    {"keeper", robotType::random}
            }
            },
            {"PassTactic", {
                    {"passOne", robotType::random},
                    {"passB", robotType::random}
            }
            },
            {"KeeperTestTactic", {
                    {"keeper", robotType::random},
                    {"atak", robotType::random}
            }
            },
            {"QualificationTactic", {
                    {"qualRole", robotType::random},
                    {"eloRlauq", robotType::random}
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
    return node;
}
