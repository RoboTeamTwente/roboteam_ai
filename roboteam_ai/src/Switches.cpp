//
// Created by baris on 15/11/18.
//

#include "Switches.h"
//  ______________________
//  |                    |
//  |   INCLUDE TACTICS  |
//  |____________________|
//

#include "roboteam_ai/src/bt/tactics/VictoryDanceTactic.h"
#include "roboteam_ai/src/bt/tactics/DefaultTactic.h"
#include "roboteam_ai/src/bt/tactics/EnterFormationTactic.h"
#include "roboteam_ai/src/bt/tactics/AvoidBallTactic.h"
#include "roboteam_ai/src/bt/tactics/PenaltyTactic.h"


//  ______________________
//  |                    |
//  |   INCLUDE SKILLS   |
//  |____________________|
//

#include "roboteam_ai/src/skills/Chip.h"
#include "roboteam_ai/src/skills/Dribble.h"
#include "roboteam_ai/src/skills/SkillGoToPos.h"
#include "roboteam_ai/src/skills/Halt.h"
#include "roboteam_ai/src/skills/Harass.h"
#include "roboteam_ai/src/skills/RotateToAngle.h"
#include "roboteam_ai/src/skills/GoToPos.h"
#include "roboteam_ai/src/skills/Keeper.h"
#include "roboteam_ai/src/skills/GetBall.h"
#include "roboteam_ai/src/skills/Attack.h"
#include "roboteam_ai/src/skills/SideAttacker.h"
#include "roboteam_ai/src/skills/Pass.h"
#include "roboteam_ai/src/skills/Receive.h"
#include <roboteam_ai/src/skills/DefendOnRobot.h>
#include "roboteam_ai/src/skills/DribbleRotate.h"
#include <roboteam_ai/src/skills/Defend.h>
#include <roboteam_ai/src/skills/GTPSpecial.h>
#include "roboteam_ai/src/skills/GoAroundPos.h"
#include "roboteam_ai/src/skills/GoBehindBall.h"
#include "roboteam_ai/src/skills/ShootPenalty.h"
#include "roboteam_ai/src/skills/ShootFreeKick.h"
#include "roboteam_ai/src/skills/DemoAttack.h"
#include "roboteam_ai/src/skills/CoachDefend.h"

//  ______________________
//  |                    |
//  | INCLUDE CONDITIONS |
//  |____________________|
//

#include "roboteam_ai/src/conditions/HasBall.hpp"
#include <roboteam_ai/src/conditions/TheyHaveBall.h>
#include <roboteam_ai/src/conditions/WeHaveBall.h>
#include <roboteam_ai/src/conditions/IsRobotClosestToBall.h>
#include <roboteam_ai/src/conditions/BallKickedToOurGoal.h>
#include <roboteam_ai/src/conditions/IsBallOnOurSide.h>
#include <roboteam_ai/src/skills/EnterFormation.h>
#include <roboteam_ai/src/skills/AvoidBall.h>
#include "roboteam_ai/src/conditions/IsDefenderGettingBall.h"

#include "roboteam_ai/src/conditions/BallInDefenseAreaAndStill.h"
#include "roboteam_ai/src/conditions/IsInDefenseArea.hpp"
#include "roboteam_ai/src/conditions/BallOutOfField.h"
#include "roboteam_ai/src/conditions/IsBeingPassedTo.h"
#include "roboteam_ai/src/conditions/IsCloseToPoint.h"
#include "roboteam_ai/src/conditions/IsBallCloseToBorder.h"
#include "roboteam_ai/src/conditions/BallNearOurGoalLineAndStill.h"
#include "roboteam_ai/src/conditions/TwoRobotBallPlacement.h"

/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */

using robotType = rtt::ai::robotDealer::RobotType;

std::vector<std::string> Switches::tacticJsonFileNames = {
        "QualificationTactic",
        "haltTactic",
        "OneAttackerTactic",
        "OneDefenderTactic",
        "TwoDefendersTactic",
        "OneAttackerOneDefenderTactic",
        "Attactic",
        "PassTactic",
        "EnterFormationTactic",
        "BallPlacementUsTactic",
        "AvoidBallTactic",
        "SingleKeeperTactic",
        "DemoAttackerTactic",
        "DemoTactic",
        "randomTactic", // used for testing, do not remove it!
        "PenaltyShootTactic",
        "PenaltyTactic",
        "FreeKickShootTactic",
        "SideAttackerTactic",
        "PassAndShootTactic",
        "coachDefenderTactic",
        "BallPlacementDoubleTactic"
};

std::vector<std::string> Switches::strategyJsonFileNames = {
        "QualificationStrategy",
        "haltStrategy",
        "KeeperStrategy",
        "DemoStrategy",
        "PassStrategy",
        "DemoTeamTwenteStrategy",
        "twoPlayerStrategyV2",
        "threePlayerStrategyV2",
        "EnterFormationStrategy",
        "BallPlacementUsStrategy",
        "BallPlacementThemStrategy",
        "randomStrategy", // used for testing, do not remove it!
        "PenaltyShootStrategy",
        "PenaltyStrategy",
        "FreeKickShootStrategy",
        "SideAttackerStrategy",
        "PassAndShootStrategy",
        "coachDefenderStrategy"
};

std::vector<std::string> Switches::keeperJsonFiles =
        {"keeperTest1",
         "SingleKeeperTactic",
         "haltTactic",
         "AvoidBallTactic"
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

    // skills (alphabetic order)

    /*
     * unused skills
     * chip
     * shootAtGoal
     * sideAttacker
     */

    map["TwoRobotBallPlacement"] = std::make_shared<rtt::ai::TwoRobotBallPlacement>(name, properties);
    map["Attack"] = std::make_shared<rtt::ai::Attack>(name, properties);
    map["AvoidBall"] = std::make_shared<rtt::ai::AvoidBall>(name, properties);
    map["CoachDefend"] = std::make_shared<rtt::ai::CoachDefend>(name, properties);
    map["GTPSpecial"] = std::make_shared<rtt::ai::GTPSpecial>(name, properties);
    map["Chip"] = std::make_shared<rtt::ai::Chip>(name,properties);
    map["Defend"] = std::make_shared<rtt::ai::Defend>(name, properties);
    map["DefendOnRobot"] = std::make_shared<rtt::ai::DefendOnRobot>(name, properties);
    map["DemoAttack"] = std::make_shared<rtt::ai::DemoAttack>(name, properties);
    map["Dribble"] = std::make_shared<rtt::ai::Dribble>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);
    map["EnterFormation"] = std::make_shared<rtt::ai::EnterFormation>(name, properties);
    map["GetBall"] = std::make_shared<rtt::ai::GetBall>(name, properties);
    map["GoAroundPos"] = std::make_shared<rtt::ai::GoAroundPos>(name, properties);
    map["GoToPos"] = std::make_shared<rtt::ai::GoToPos>(name, properties);
    map["Halt"] = std::make_shared<rtt::ai::Halt>(name, properties);
    map["Harass"] = std::make_shared<rtt::ai::Harass>(name, properties);
    map["InterceptBall"] = std::make_shared<rtt::ai::InterceptBall>(name, properties);
    map["Keeper"] = std::make_shared<rtt::ai::Keeper>(name, properties);
    map["Kick"] = std::make_shared<rtt::ai::Kick>(name, properties);
    map["Pass"] = std::make_shared<rtt::ai::Pass>(name, properties);
    map["Receive"] = std::make_shared<rtt::ai::Receive>(name, properties);
    map["RotateToAngle"] = std::make_shared<rtt::ai::RotateToAngle>(name, properties);
    map["SkillGoToPos"] = std::make_shared<rtt::ai::SkillGoToPos>(name, properties);
    map["SideAttacker"] = std::make_shared<rtt::ai::SideAttacker>(name, properties);
    map["GoBehindBall"] = std::make_shared<rtt::ai::GoBehindBall>(name, properties);
    map["ShootPenalty"] = std::make_shared<rtt::ai::ShootPenalty>(name, properties);
    map["ShootFreeKick"] = std::make_shared<rtt::ai::ShootFreeKick>(name, properties);

    // conditions (alphabetic order)
    map["BallKickedToOurGoal"] = std::make_shared<rtt::ai::BallKickedToOurGoal>(name, properties);
    map["BallInDefenseAreaAndStill"] = std::make_shared<rtt::ai::BallInDefenseAreaAndStill>(name, properties);
    map["BallNearOurGoalLineAndStill"] = std::make_shared<rtt::ai::BallNearOurGoalLineAndStill>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);
    map["HasBall"] = std::make_shared<rtt::ai::HasBall>(name, properties);
    map["IsBallCloseToBorder"] = std::make_shared<rtt::ai::IsBallCloseToBorder>(name, properties);
    map["IsBallOnOurSide"] = std::make_shared<rtt::ai::IsBallOnOurSide>(name, properties);
    map["IsRobotClosestToBall"] = std::make_shared<rtt::ai::IsRobotClosestToBall>(name, properties);
    map["IsInDefenseArea"] = std::make_shared<rtt::ai::IsInDefenseArea>(name, properties);
    map["TheyHaveBall"] = std::make_shared<rtt::ai::TheyHaveBall>(name, properties);
    map["BallOutOfField"] = std::make_shared<rtt::ai::BallOutOfField>(name, properties);
    map["WeHaveBall"] = std::make_shared<rtt::ai::WeHaveBall>(name, properties);
    map["IsBeingPassedTo"] = std::make_shared<rtt::ai::IsBeingPassedTo>(name, properties);
    map["IsCloseToPoint"] = std::make_shared<rtt::ai::IsCloseToPoint>(name, properties);
    map["IsBallOnOurSide"] = std::make_shared<rtt::ai::IsBallOnOurSide>(name, properties);
    map["IsDefenderGettingBall"] = std::make_shared<rtt::ai::IsDefenderGettingBall>(name,properties);
    map["BallInDefenseAreaAndStill"] = std::make_shared<rtt::ai::BallInDefenseAreaAndStill>(name, properties);
    map["IsInDefenseArea"] = std::make_shared<rtt::ai::IsInDefenseArea>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);

    if (map.find(name) != map.end()) {
        return map[name];
    }
    else {

        ROS_ERROR("\n\n\nTHE LEAF IS NOT REGISTERED IN SWITCHES:   %s\n\n\n", name.c_str());
        return bt::Node::Ptr();
    }
}

/// If you made a tactic node for a new tactic this is where you add that
bt::Node::Ptr Switches::tacticSwitch(std::string name, bt::Blackboard::Ptr properties) {

    std::map<std::string, std::map<std::string, robotType>> tactics = {

            {"haltTactic", {
                    {"halt0", robotType::RANDOM},
                    {"halt1", robotType::RANDOM},
                    {"halt2", robotType::RANDOM},
                    {"halt3", robotType::RANDOM},
                    {"halt4", robotType::RANDOM},
                    {"halt5", robotType::RANDOM},
                    {"halt6", robotType::RANDOM}
            }
            },

            {"OneAttackerTactic", {
                    {"attacker", robotType::CLOSE_TO_THEIR_GOAL}
            }
            },
            {"OneAttackerOneDefenderTactic", {
                    {"defender", robotType::CLOSE_TO_OUR_GOAL},
                    {"attacker", robotType::CLOSE_TO_THEIR_GOAL}
            }
            },
            {"OneDefenderTactic", {
                    {"defender", robotType::CLOSE_TO_THEIR_GOAL}
            }
            },
            {"TwoDefendersTactic", {
                    {"defender1", robotType::CLOSE_TO_OUR_GOAL},
                    {"defender2", robotType::CLOSE_TO_OUR_GOAL},
            }
            },
            {"Attactic", {
                    {"atak", robotType::RANDOM}
            }
            },
            {"PassTactic", {
                    {"passer", robotType::CLOSE_TO_BALL},
                    {"receiver", robotType::RANDOM}
            }
            },
            {"QualificationTactic", {
                    {"qualRole", robotType::RANDOM},
                    {"eloRlauq", robotType::RANDOM}
            }
            },
            {"randomTactic", {
                    {"random1", robotType::RANDOM},
                    {"random2", robotType::RANDOM},
                    {"random3", robotType::RANDOM},
                    {"random4", robotType::RANDOM},
                    {"random5", robotType::RANDOM},
                    {"random6", robotType::RANDOM},
                    {"random7", robotType::RANDOM}
            }
            },
            {"BallPlacementUsTactic", {
                    {"BallPlacementBot", robotType::CLOSE_TO_BALL}
            }
            },
            {"BallPlacementDoubleTactic", {
                    {"BallPlacementPasser", robotType::CLOSE_TO_BALL},
                    {"BallPlacementReceiver", robotType::BALL_PLACEMENT_RECEIVER},
                    {"avoid1", robotType::RANDOM},
                    {"avoid2", robotType::RANDOM},
                    {"avoid3", robotType::RANDOM},
                    {"avoid4", robotType::RANDOM},
                    {"avoid5", robotType::RANDOM}
            }
            },
            {"SingleKeeperTactic", {
                    {"Keeper", robotType::CLOSE_TO_OUR_GOAL}
            }
            },
            {"DemoAttackerTactic", {
                    {"demoAttacker", robotType::CLOSE_TO_THEIR_GOAL}
            }
            },
            {"DemoTactic", {
                    {"demoAttacker", robotType::CLOSE_TO_THEIR_GOAL},
                    {"demoKeeper", robotType::CLOSE_TO_OUR_GOAL}
            }
            },
            {"SingleKeeperTactic", {
                    {"Keeper", robotType::CLOSE_TO_OUR_GOAL}
            }
            },
            {"SideAttackerTactic", {
                    {"sideAttacker1", robotType::CLOSE_TO_THEIR_GOAL},
                    {"sideAttacker2", robotType::CLOSE_TO_THEIR_GOAL},
                    {"sideAttacker3", robotType::CLOSE_TO_THEIR_GOAL},
                    {"sideAttacker4", robotType::CLOSE_TO_THEIR_GOAL}

            }
            },
            {"PassAndShootTactic", {
                    {"midfielder1", robotType::CLOSE_TO_BALL},
                    {"sideAttacker1", robotType::CLOSE_TO_THEIR_GOAL},
                    {"sideAttacker2", robotType::CLOSE_TO_THEIR_GOAL}

            }

            },
            {"PenaltyShootTactic", {
                    {"shooter", robotType::RANDOM}
            }
            },
            {"PenaltyTactic", {
                    {"shooter", robotType::RANDOM}
            }
            },
            {"FreeKickShootTactic", {
                    {"freeShooter", robotType::RANDOM}
            }
            },

            {"coachDefenderTactic",
             {
                     {"def1", robotType::RANDOM},
                     {"def2", robotType::RANDOM},
                     {"def3", robotType::RANDOM},
                     {"def4", robotType::RANDOM},
                     {"def5", robotType::RANDOM},
                     {"def6", robotType::RANDOM},
                     {"def7", robotType::RANDOM}
             }
            }
    };
    runErrorHandler(tactics);

    bt::Node::Ptr node;

    if (name == "VerySpecialTacticThatWouldRequireSpecialClass")
        node = std::make_shared<bt::VictoryDanceTactic>("VerySpecialTacticThatWouldRequireSpecialClass", properties);
    else if (tactics.find(name) != tactics.end())
        node = std::make_shared<bt::DefaultTactic>(name, properties, tactics[name]);
    else if (name == "EnterFormationTactic")
        node = std::make_shared<bt::EnterFormationTactic>("EnterFormationTactic", properties);
    else if (name == "AvoidBallTactic")
        node = std::make_shared<bt::AvoidBallTactic>("AvoidBallTactic", properties);
    else if (name == "victoryDanceTactic")
        node = std::make_shared<bt::VictoryDanceTactic>("victoryDanceTactic", properties);
    else if (name == "PenaltyTactic")
        node = std::make_shared<bt::PenaltyTactic>("PenaltyTactic", properties);
    else
        ROS_ERROR("\n\n\nTHE TACTIC DOES NOT HAVE ROBOTS SPECIFIED IN THE SWITCHES:    %s\n\n\n", name.c_str());
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
