/*
 *
 * Analysis Reports are a summary of the world state, which includes the things that are important to us.
 *
 *
 */

#ifndef ROBOTEAM_AI_ANALYSISREPORT_H
#define ROBOTEAM_AI_ANALYSISREPORT_H

#include "RobotDanger.h"
#include "roboteam_proto/WorldRobot.pb.h"
#include "world/Robot.h"

namespace rtt::ai::analysis {

// define some play styles to influence our decision making
enum BallPossession : short { THEY_HAVE_BALL, DEFENSIVE_NEUTRAL, NEUTRAL, OFFENSIVE_NEUTRAL, WE_HAVE_BALL };

struct AnalysisReport {
    bool reportForUs = true;
    std::vector<std::pair<world::Robot::RobotPtr, RobotDanger>> theirRobotSortedOnDanger;
    std::vector<std::pair<world::Robot::RobotPtr, RobotDanger>> ourRobotsSortedOnDanger;

    BallPossession ballPossession = NEUTRAL;
    double ourDistanceToGoalAvg = 0.0;
    double theirDistanceToGoalAvg = 0.0;

    RobotDanger getRobotDangerForId(int id, bool ourTeam);
};

}  // namespace rtt::ai::analysis

#endif  // ROBOTEAM_AI_ANALYSISREPORT_H
