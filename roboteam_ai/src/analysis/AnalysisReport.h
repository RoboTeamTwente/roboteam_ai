/*
 *
 * Analysis Reports are a summary of the world state, which includes the things that are important to us.
 *
 *
 */

#ifndef ROBOTEAM_AI_ANALYSISREPORT_H
#define ROBOTEAM_AI_ANALYSISREPORT_H

#include <roboteam_msgs/WorldRobot.h>
#include "RobotDanger.h"

namespace rtt {
namespace ai {
namespace analysis {

// define some play styles to influence our decision making
enum playStyle {
    DEFEND_WITH_ALL                 = 0, // all robots defend
    DEFEND_WITH_ALL_MIDFIELDERS     = 1, // defenders and midfielders or an attacker defend
    DEFEND_WITH_SOME_MIDFIELDERS    = 2, // defenders and 1/2 of midfielders or an attacker defend
    UNSURE_DEFENSIVE                = 3, // standard formation but relatively close to our goal
    UNSURE_OFFENSIVE                = 4, // standard formation but relatively more wide
    ATTACK_WITH_SOME_MIDFIELDERS    = 5, // some midfielders attack
    ATTACK_WITH_ALL_MIDFIELDERS     = 6, // all midfielders and attackers attack
    MAKE_THEM_PAY                   = 7, // attack with attackers, midfielders and two defenders
};

struct AnalysisReport {
    bool reportForUs = true;
    std::vector<std::pair<roboteam_msgs::WorldRobot, RobotDanger>> theirRobotSortedOnDanger;
    std::vector<std::pair<roboteam_msgs::WorldRobot, RobotDanger>> ourRobotsSortedOnDanger;

    playStyle recommendedPlayStyle;
    double ballPossession = 0.0;
    double ourDistanceToGoalAvg = 0.0;
    double theirDistanceToGoalAvg = 0.0;

    RobotDanger getRobotDangerForId(int id, bool ourTeam);

};

} // analysis
} // ai
} // rtt

#endif //ROBOTEAM_AI_ANALYSISREPORT_H
