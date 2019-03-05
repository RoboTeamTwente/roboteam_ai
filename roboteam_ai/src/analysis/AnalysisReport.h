//
// Created by mrlukasbos on 5-3-19.
//

#ifndef ROBOTEAM_AI_ANALYSISREPORT_H
#define ROBOTEAM_AI_ANALYSISREPORT_H

#include <roboteam_msgs/WorldRobot.h>

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
    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> robotSortedOnDanger;
    playStyle recommendedPlayStyle;

};

} // analysis
} // ai
} // rtt

#endif //ROBOTEAM_AI_ANALYSISREPORT_H
