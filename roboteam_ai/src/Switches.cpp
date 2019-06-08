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

#include "roboteam_ai/src/skills/Chip.h"
#include "roboteam_ai/src/skills/Dribble.h"
#include "roboteam_ai/src/skills/DribbleForward.h"
#include "roboteam_ai/src/skills/gotopos/SkillGoToPos.h"
#include "roboteam_ai/src/skills/Halt.h"
#include "roboteam_ai/src/skills/Harass.h"
#include "roboteam_ai/src/skills/RotateToAngle.h"
#include "roboteam_ai/src/skills/gotopos/GoToPos.h"
#include "roboteam_ai/src/skills/Keeper.h"
#include "roboteam_ai/src/skills/OpponentKeeper.h"
#include "roboteam_ai/src/skills/GetBall.h"
#include "roboteam_ai/src/skills/Attack.h"
#include "roboteam_ai/src/skills/SideAttacker.h"
#include "roboteam_ai/src/skills/Pass.h"
#include "roboteam_ai/src/skills/Receive.h"
#include "roboteam_ai/src/skills/DribbleRotate.h"
#include <roboteam_ai/src/skills/gotopos/GTPSpecial.h>
#include "roboteam_ai/src/skills/gotopos/GoAroundPos.h"
#include "roboteam_ai/src/skills/gotopos/GTPWithBall.h"
#include "roboteam_ai/src/skills/gotopos/GoBehindBall.h"
#include "roboteam_ai/src/skills/ShootPenalty.h"
#include "roboteam_ai/src/skills/ShootFreeKick.h"
#include "roboteam_ai/src/skills/DemoAttack.h"
#include <roboteam_ai/src/skills/MidFieldHarasser.h>
#include "roboteam_ai/src/skills/ReflectKick.h"
#include "roboteam_ai/src/skills/InterceptRobot.hpp"
#include "roboteam_ai/src/skills/InterceptBall.h"
#include "roboteam_ai/src/skills/OpponentInterceptBall.h"
#include "roboteam_ai/src/skills/CoachDefend.h"
#include "roboteam_ai/src/skills/formations/PenaltyFormation.h"
#include "roboteam_ai/src/skills/formations/FreeKickFormation.h"
#include "roboteam_ai/src/skills/formations/DefendFreeKick.h"
#include "roboteam_ai/src/skills/ActiveStop.h"
#include "roboteam_ai/src/skills/SlingShot.h"
#include <roboteam_ai/src/skills/PenaltyKeeper.h>
#include <roboteam_ai/src/skills/DriveWithInterface.h>
#include <roboteam_ai/src/skills/BallPlacementWithInterface.h>
#include <roboteam_ai/src/skills/MidFieldHarasser.h>
#include <roboteam_ai/src/skills/Wait.h>


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
#include <roboteam_ai/src/conditions/BallKickedToTheirGoal.h>
#include <roboteam_ai/src/conditions/IsBallOnOurSide.h>
#include <roboteam_ai/src/skills/formations/KickOffUsFormation.h>
#include <roboteam_ai/src/skills/AvoidBall.h>
#include "roboteam_ai/src/conditions/ShouldHandleBall.h"
#include <roboteam_ai/src/skills/formations/TimeoutFormation.h>
#include <roboteam_ai/src/bt/RoleDivider.h>
#include <roboteam_ai/src/skills/formations/KickOffThemFormation.h>
#include <roboteam_ai/src/skills/ballPlacement/BallPlacementReceive.h>
#include <roboteam_ai/src/conditions/CanPlay.h>
#include <roboteam_ai/src/conditions/RobotOutside.h>
#include <roboteam_ai/src/conditions/RefStateIsNormalPlay.h>
#include <roboteam_ai/src/conditions/RefBallIsMoving.h>
#include <roboteam_ai/src/skills/ballPlacement/BallPlacementPass.h>
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


#include "roboteam_ai/src/conditions/BallInDefenseAreaAndStill.h"
#include "roboteam_ai/src/conditions/IsInDefenseArea.hpp"
#include "roboteam_ai/src/conditions/BallOutOfField.h"
#include "roboteam_ai/src/conditions/IsBeingPassedTo.h"
#include "roboteam_ai/src/conditions/IsCloseToPoint.h"
#include "roboteam_ai/src/conditions/IsBallCloseToBorder.h"
#include "roboteam_ai/src/conditions/BallNearOurGoalLineAndStill.h"
#include "roboteam_ai/src/conditions/TwoRobotBallPlacement.h"
#include "roboteam_ai/src/conditions/HasClearShot.h"
#include "roboteam_ai/src/conditions/IsOnPassLine.h"

/**
 * When you want to add a new class to the ai, you need to change this file so the first two vector have the FILE NAMES
 * of the json trees you added
 *
 * Then depending on the type of node you made you need to add them to the switches below with the names that are
 * specified in the json trees. These are usually the same name as the classes you make for that tactic.
 */

using robotType = rtt::ai::robotDealer::RobotType;

std::vector<std::string> Switches::tacticJsonFileNames = {
        "avoid_tactic",
        "free_kick_formation_tactic",
        "free_kick_shoot_tactic",
        "free_kick_them_tactic",
        "halt_tactic",
        "kickoff_shoot_tactic",
        "kickoff_them_tactic",
        "kickoff_them_formation_tactic",
        "kickoff_us_formation_tactic",
        "one_robot_ballplacement_tactic",
        "penalty_them_tactic",
        "prepare_penalty_us_tactic",
        "shoot_penalty_us_tactic",
        "stop_tactic",
        "normal_play_defense_tactic",
        "normal_play_midfield_tactic",
        "normal_play_offense_tactic",
        "time_out_tactic",
        "two_robot_ballplacement_tactic",
        "calibration_tactic",
        "follow_interface_tactic",
        "ball_placement_interface_tactic",
        "attackerTestTactic"
};

std::vector<std::string> Switches::strategyJsonFileNames = {
        "kickoff_them_formation_strategy",
        "kickoff_them_strategy",
        "kickoff_us_formation_strategy",
        "time_out_strategy",
        "ball_placement_us_strategy",
        "ball_placement_them_strategy",
        "stop_strategy",
        "halt_strategy",
        "normal_play_strategy",
        "penalty_us_shoot_strategy",
        "penalty_us_prepare_strategy",
        "penalty_them_strategy",
        "free_kick_formation_strategy",
        "free_kick_shoot_strategy",
        "free_kick_them_strategy",
        "kickoff_shoot_strategy",
        "calibration_strategy",
        "interface_drive_strategy",
        "interface_ball_placement_strategy",
        "attackTestStrategy"
};

std::vector<std::string> Switches::keeperJsonFiles = {
        "opponent_keeper_default_tactic",
        "keeper_default_tactic",
        "keeper_halt_tactic",
        "keeper_avoid_tactic",
        "keeper_time_out_tactic",
        "keeper_formation_tactic",
        "keeper_penalty_tactic"
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

    map["TwoRobotBallPlacement"] = std::make_shared<rtt::ai::TwoRobotBallPlacement>(name, properties);
    map["Attack"] = std::make_shared<rtt::ai::Attack>(name, properties);
    map["AvoidBall"] = std::make_shared<rtt::ai::AvoidBall>(name, properties);
    map["CoachDefend"] = std::make_shared<rtt::ai::CoachDefend>(name, properties);
    map["GTPSpecial"] = std::make_shared<rtt::ai::GTPSpecial>(name, properties);
    map["Chip"] = std::make_shared<rtt::ai::Chip>(name,properties);
    map["DemoAttack"] = std::make_shared<rtt::ai::DemoAttack>(name, properties);
    map["Dribble"] = std::make_shared<rtt::ai::Dribble>(name, properties);
    map["DribbleForward"] = std::make_shared<rtt::ai::DribbleForward>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);
    map["TimeoutFormation"] = std::make_shared<rtt::ai::TimeoutFormation>(name, properties);
    map["KickOffUsFormation"] = std::make_shared<rtt::ai::KickOffUsFormation>(name, properties);
    map["KickOffThemFormation"] = std::make_shared<rtt::ai::KickOffThemFormation>(name, properties);

    map["GetBall"] = std::make_shared<rtt::ai::GetBall>(name, properties);
    map["GoAroundPos"] = std::make_shared<rtt::ai::GoAroundPos>(name, properties);
    map["GoToPos"] = std::make_shared<rtt::ai::SkillGoToPos>(name, properties);
    map["Halt"] = std::make_shared<rtt::ai::Halt>(name, properties);
    map["Harass"] = std::make_shared<rtt::ai::Harass>(name, properties);
    map["InterceptBall"] = std::make_shared<rtt::ai::InterceptBall>(name, properties);
    map["OpponentInterceptBall"] = std::make_shared<rtt::ai::OpponentInterceptBall>(name, properties);
    map["InterceptRobot"] = std::make_shared<rtt::ai::InterceptRobot>(name, properties);
    map["Keeper"] = std::make_shared<rtt::ai::Keeper>(name, properties);
    map["OpponentKeeper"] = std::make_shared<rtt::ai::OpponentKeeper>(name, properties);
    map["Kick"] = std::make_shared<rtt::ai::Kick>(name, properties);
    map["Pass"] = std::make_shared<rtt::ai::Pass>(name, properties);
    map["Receive"] = std::make_shared<rtt::ai::Receive>(name, properties);
    map["RotateToAngle"] = std::make_shared<rtt::ai::RotateToAngle>(name, properties);
    map["SkillGoToPos"] = std::make_shared<rtt::ai::SkillGoToPos>(name, properties);
    map["SideAttacker"] = std::make_shared<rtt::ai::SideAttacker>(name, properties);
    map["GoBehindBall"] = std::make_shared<rtt::ai::GoBehindBall>(name, properties);
    map["ShootPenalty"] = std::make_shared<rtt::ai::ShootPenalty>(name, properties);
    map["ShootFreeKick"] = std::make_shared<rtt::ai::ShootFreeKick>(name, properties);
    map["SlingShot"] = std::make_shared<rtt::ai::SlingShot>(name, properties);
    map["PenaltyFormation"] = std::make_shared<rtt::ai::PenaltyFormation>(name, properties);
    map["ActiveStop"] = std::make_shared<rtt::ai::ActiveStop>(name, properties);
    map["ReflectKick"] = std::make_shared<rtt::ai::ReflectKick>(name, properties);
    map["MidFieldHarasser"] = std::make_shared<rtt::ai::MidFieldHarasser>(name, properties);
    map["DribbleRotate"] = std::make_shared<rtt::ai::DribbleRotate>(name, properties);
    map["PenaltyKeeper"] = std::make_shared<rtt::ai::PenaltyKeeper>(name, properties);
    map["FreeKickFormation"] = std::make_shared<rtt::ai::FreeKickFormation>(name, properties);
    map["DefendFreeKick"] = std::make_shared<rtt::ai::DefendFreeKick>(name, properties);
    map["GTPWithBall"] = std::make_shared<rtt::ai::GTPWithBall>(name, properties);
    map["BallPlacementReceive"] = std::make_shared<rtt::ai::BallPlacementReceive>(name, properties);
    map["BallPlacementPass"] = std::make_shared<rtt::ai::BallPlacementPass>(name, properties);
    map["DriveWithInterface"] = std::make_shared<rtt::ai::DriveWithInterface>(name, properties);
    map["BallPlacementWithInterface"] = std::make_shared<rtt::ai::BallPlacementWithInterface>(name, properties);
    map["Wait"] = std::make_shared<rtt::ai::Wait>(name, properties);



    // conditions (alphabetic order)
    map["BallKickedToOurGoal"] = std::make_shared<rtt::ai::BallKickedToOurGoal>(name, properties);
    map["BallKickedToTheirGoal"] = std::make_shared<rtt::ai::BallKickedToTheirGoal>(name, properties);
    map["BallInDefenseAreaAndStill"] = std::make_shared<rtt::ai::BallInDefenseAreaAndStill>(name, properties);
    map["BallNearOurGoalLineAndStill"] = std::make_shared<rtt::ai::BallNearOurGoalLineAndStill>(name, properties);
    map["CanPlay"] = std::make_shared<rtt::ai::CanPlay>(name, properties);
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
    map["ShouldHandleBall"] = std::make_shared<rtt::ai::ShouldHandleBall>(name,properties);
    map["BallInDefenseAreaAndStill"] = std::make_shared<rtt::ai::BallInDefenseAreaAndStill>(name, properties);
    map["IsInDefenseArea"] = std::make_shared<rtt::ai::IsInDefenseArea>(name, properties);
    map["HasClearShot"] = std::make_shared<rtt::ai::HasClearShot>(name, properties);
    map["IsOnPassLine"] = std::make_shared<rtt::ai::IsOnPassLine>(name, properties);
    map["RobotOutside"] = std::make_shared<rtt::ai::RobotOutside>(name, properties);
    map["RefStateIsNormalPlay"] = std::make_shared<rtt::ai::RefStateIsNormalPlay>(name, properties);
    map["RefBallIsMoving"] = std::make_shared<rtt::ai::RefBallIsMoving>(name, properties);

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

            // Keeper tactics
            {"keeper_default_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            {"opponent_keeper_default_tactic", {{"Keeper", robotType::CLOSE_TO_THEIR_GOAL}}},
            {"keeper_avoid_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            {"keeper_halt_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            {"keeper_time_out_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            {"keeper_formation_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            {"keeper_penalty_tactic", {{"Keeper", robotType::CLOSE_TO_OUR_GOAL}}},
            // General tactics
            {"halt_tactic", {
                    {"halt0", robotType::RANDOM},
                    {"halt1", robotType::RANDOM},
                    {"halt2", robotType::RANDOM},
                    {"halt3", robotType::RANDOM},
                    {"halt4", robotType::RANDOM},
                    {"halt5", robotType::RANDOM},
                    {"halt6", robotType::RANDOM},
                    {"halt7", robotType::RANDOM}
            }
            },

            {"avoid_tactic", {
                    {"avoid1", robotType::RANDOM},
                    {"avoid2", robotType::RANDOM},
                    {"avoid3", robotType::RANDOM},
                    {"avoid4", robotType::RANDOM},
                    {"avoid5", robotType::RANDOM},
                    {"avoid6", robotType::RANDOM},
                    {"avoid7", robotType::RANDOM},
                    {"avoid8", robotType::RANDOM}
            }
            },
            {"follow_interface_tactic", {
                    {"follow_interface", robotType::RANDOM},
                    {"f1", robotType::RANDOM},
                    {"f2", robotType::RANDOM},
                    {"f3", robotType::RANDOM},
                    {"f4", robotType::RANDOM},
                    {"f5", robotType::RANDOM},
                    {"f6", robotType::RANDOM},
                    {"f7", robotType::RANDOM}
            }
            },
            {"ball_placement_interface_tactic", {
                    {"follow_interface", robotType::RANDOM},
                    {"f1", robotType::RANDOM},
                    {"f2", robotType::RANDOM},
                    {"f3", robotType::RANDOM},
                    {"f4", robotType::RANDOM},
                    {"f5", robotType::RANDOM},
                    {"f6", robotType::RANDOM},
                    {"f7", robotType::RANDOM}
            }
            },

            {"time_out_tactic", {
                    {"timeout1", robotType::RANDOM},
                    {"timeout2", robotType::RANDOM},
                    {"timeout3", robotType::RANDOM},
                    {"timeout4", robotType::RANDOM},
                    {"timeout5", robotType::RANDOM},
                    {"timeout6", robotType::RANDOM},
                    {"timeout7", robotType::RANDOM},
                    {"timeout8", robotType::RANDOM}
            }
            },

            {"prepare_penalty_us_tactic", {
                    {"shooter", robotType::WORKING_GENEVA},
                    {"pa1", robotType::RANDOM},
                    {"pa2", robotType::RANDOM},
                    {"pa3", robotType::RANDOM},
                    {"pa4", robotType::RANDOM},
                    {"pa5", robotType::RANDOM},
                    {"pa6", robotType::RANDOM},
                    {"pa7", robotType::RANDOM}
            }
            },

            {"kickoff_them_tactic", {
                  {"ko0", robotType::CLOSE_TO_BALL},
                  {"ko1", robotType::RANDOM},
                  {"ko2", robotType::RANDOM},
                  {"ko3", robotType::RANDOM},
                  {"ko4", robotType::RANDOM},
                  {"ko5", robotType::RANDOM},
                  {"ko6", robotType::RANDOM},
                  {"ko7", robotType::RANDOM}
            }
            },

            {"kickoff_shoot_tactic", {
                     {"kicker", robotType::CLOSE_TO_BALL},
                     {"assist1", robotType::CLOSE_TO_THEIR_GOAL},
                     {"assist2", robotType::CLOSE_TO_THEIR_GOAL},
                     {"ko1", robotType::RANDOM},
                     {"ko2", robotType::RANDOM},
                     {"ko3", robotType::RANDOM},
                     {"ko4", robotType::RANDOM},
                     {"ko5", robotType::RANDOM}
             }
            },

            {"stop_tactic", {
                    {"a1", robotType::CLOSE_TO_BALL},
                    {"a2", robotType::CLOSE_TO_BALL},
                    {"p1", robotType::RANDOM},
                    {"p2", robotType::RANDOM},
                    {"p3", robotType::RANDOM},
                    {"p4", robotType::RANDOM},
                    {"p5", robotType::RANDOM},
                    {"p6", robotType::RANDOM}
            }
            },

            {"one_robot_ballplacement_tactic", {
                    {"ballplacementbot", robotType::CLOSE_TO_BALL},
                    {"avoid1", robotType::RANDOM},
                    {"avoid2", robotType::RANDOM},
                    {"avoid3", robotType::RANDOM},
                    {"avoid4", robotType::RANDOM},
                    {"avoid5", robotType::RANDOM},
                    {"avoid6", robotType::RANDOM},
                    {"avoid7", robotType::RANDOM}
            }
            },
            {"kickoff_them_formation_tactic", {
                    {"kickoff1", robotType::RANDOM},
                    {"kickoff2", robotType::RANDOM},
                    {"kickoff3", robotType::RANDOM},
                    {"kickoff4", robotType::RANDOM},
                    {"kickoff5", robotType::RANDOM},
                    {"kickoff6", robotType::RANDOM},
                    {"kickoff7", robotType::RANDOM},
                    {"kickoff8", robotType::RANDOM}
            }
            },

            {"kickoff_us_formation_tactic", {
                    {"kickoff1", robotType::RANDOM},
                    {"kickoff2", robotType::RANDOM},
                    {"kickoff3", robotType::RANDOM},
                    {"kickoff4", robotType::RANDOM},
                    {"kickoff5", robotType::RANDOM},
                    {"kickoff6", robotType::RANDOM},
                    {"kickoff7", robotType::RANDOM},
                    {"kickoff8", robotType::RANDOM}
            }
            },

            {"one_robot_ballplacement_tactic", {
                    {"ballplacementbot", robotType::CLOSE_TO_BALL},
                    {"avoid1", robotType::RANDOM},
                    {"avoid2", robotType::RANDOM},
                    {"avoid3", robotType::RANDOM},
                    {"avoid4", robotType::RANDOM},
                    {"avoid5", robotType::RANDOM},
                    {"avoid6", robotType::RANDOM},
                    {"avoid7", robotType::RANDOM}
            }
            },

            {"two_robot_ballplacement_tactic", {
                    {"ball_placement_passer", robotType::CLOSE_TO_BALL},
                    {"ball_placement_receiver", robotType::BALL_PLACEMENT_RECEIVER},
                    {"avoid1", robotType::RANDOM},
                    {"avoid2", robotType::RANDOM},
                    {"avoid3", robotType::RANDOM},
                    {"avoid4", robotType::RANDOM},
                    {"avoid5", robotType::RANDOM},
                    {"avoid6", robotType::RANDOM}
            }
            },
            {"free_kick_formation_tactic", {
                    {"kicker", robotType::RANDOM},
                    {"f1", robotType::RANDOM},
                    {"f2", robotType::RANDOM},
                    {"f3", robotType::RANDOM},
                    {"f4", robotType::RANDOM},
                    {"f5", robotType::RANDOM},
                    {"f6", robotType::RANDOM},
                    {"f7", robotType::RANDOM}
            }
            },
            {"free_kick_shoot_tactic", {
                    {"kickerChip", robotType::CLOSE_TO_BALL},
                    {"f1", robotType::RANDOM},
                    {"f2", robotType::RANDOM},
                    {"f3", robotType::RANDOM},
                    {"f4", robotType::RANDOM},
                    {"f5", robotType::RANDOM},
                    {"f6", robotType::RANDOM},
                    {"f7", robotType::RANDOM}
            }
            },
            {"free_kick_them_tactic", {
                    {"line7", robotType::CLOSE_TO_BALL},
                    {"line6", robotType::RANDOM},
                    {"line1", robotType::RANDOM},
                    {"line2", robotType::RANDOM},
                    {"line4", robotType::RANDOM},
                    {"line5", robotType::RANDOM},
                    {"line3", robotType::RANDOM},
                    {"line8", robotType::RANDOM}
            }
            },
            {"penalty_them_tactic", {
                    {"a1", robotType::RANDOM},
                    {"a2", robotType::RANDOM},
                    {"a3", robotType::RANDOM},
                    {"a4", robotType::RANDOM},
                    {"a5", robotType::RANDOM},
                    {"a6", robotType::RANDOM},
                    {"a7", robotType::RANDOM},
                    {"a8", robotType::RANDOM}

            }
            },
            {"normal_play_defense_tactic",
             {
                     {"d1", robotType::CLOSE_TO_OUR_GOAL},
                     {"d2", robotType::CLOSE_TO_OUR_GOAL},
                     {"d3", robotType::CLOSE_TO_OUR_GOAL},
                     {"d4", robotType::CLOSE_TO_OUR_GOAL},
                     {"d5", robotType::CLOSE_TO_OUR_GOAL},
                     {"d6", robotType::CLOSE_TO_OUR_GOAL},
                     {"d7", robotType::CLOSE_TO_OUR_GOAL},
                     {"d8", robotType::CLOSE_TO_OUR_GOAL}
             }
            },
            {"normal_play_midfield_tactic",
             {
                     {"m1", robotType::CLOSE_TO_OUR_GOAL},
                     {"m2", robotType::CLOSE_TO_OUR_GOAL},
                     {"m3", robotType::CLOSE_TO_OUR_GOAL},
                     {"m4", robotType::CLOSE_TO_OUR_GOAL},
                     {"m5", robotType::CLOSE_TO_OUR_GOAL},
                     {"m6", robotType::CLOSE_TO_OUR_GOAL},
                     {"m7", robotType::CLOSE_TO_OUR_GOAL},
                     {"m8", robotType::CLOSE_TO_OUR_GOAL}
             }
            },
            {"normal_play_offense_tactic",
             {
                     {"o1", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o2", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o3", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o4", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o5", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o6", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o7", robotType::CLOSE_TO_THEIR_GOAL},
                     {"o8", robotType::CLOSE_TO_THEIR_GOAL}
             }
            },
            {"test_pass_tactic",
             {
                    {"attacker1", robotType::CLOSE_TO_THEIR_GOAL},
                    {"attacker2", robotType::CLOSE_TO_THEIR_GOAL},
                     {"receive1", robotType::RANDOM},
                     {"receive2", robotType::RANDOM},
                     {"receive3", robotType::RANDOM},
                     {"receive4", robotType::RANDOM},
                     {"receive5", robotType::RANDOM},
                     {"receive6", robotType::RANDOM}
             }
            },
            {"shoot_penalty_us_tactic", {
                    {"penaltyShooter", robotType::CLOSE_TO_BALL},
                    {"paa1", robotType::RANDOM},
                    {"paa2", robotType::RANDOM},
                    {"paa3", robotType::RANDOM},
                    {"paa4", robotType::RANDOM},
                    {"paa5", robotType::RANDOM},
                    {"paa6", robotType::RANDOM},
                    {"paa7", robotType::RANDOM}
            }
            },
            {"calibration_tactic", {
                    {"c1", robotType::CLOSE_TO_BALL},
                    {"c2", robotType::RANDOM},
                    {"c3", robotType::RANDOM},
                    {"c4", robotType::RANDOM},
                    {"c5", robotType::RANDOM},
                    {"c6", robotType::RANDOM},
                    {"c7", robotType::RANDOM},
                    {"c8", robotType::RANDOM}
            }
            },
            {"attackerTestTactic",{
                    {"attackTest", robotType::CLOSE_TO_BALL},
                    {"halt1",robotType::RANDOM},
                    {"halt2",robotType::RANDOM},
                    {"halt3",robotType::RANDOM},
                    {"halt4",robotType::RANDOM},
                    {"halt5",robotType::RANDOM},
                    {"halt6",robotType::RANDOM},
                    {"halt7",robotType::RANDOM}
            }
            }
    };
//    runErrorHandler(tactics);

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
