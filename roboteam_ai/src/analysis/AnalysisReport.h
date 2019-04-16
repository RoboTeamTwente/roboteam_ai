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
#include "../world/WorldData.h"

namespace rtt {
namespace ai {
namespace analysis {

// define some play styles to influence our decision making
enum BallPossession : short {
    THEY_HAVE_BALL,
    NEUTRAL,
    WE_HAVE_BALL
};

struct AnalysisReport {
    bool reportForUs = true;
    std::vector<std::pair<world::Robot, RobotDanger>> theirRobotSortedOnDanger;
    std::vector<std::pair<world::Robot, RobotDanger>> ourRobotsSortedOnDanger;

    BallPossession ballPossession = NEUTRAL;
    double ourDistanceToGoalAvg = 0.0;
    double theirDistanceToGoalAvg = 0.0;

    RobotDanger getRobotDangerForId(int id, bool ourTeam);

};

} // analysis
} // ai
} // rtt

#endif //ROBOTEAM_AI_ANALYSISREPORT_H
