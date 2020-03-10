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


namespace rtt::ai::analysis {

// define some play styles to influence our decision making
enum BallPossession : short { THEY_HAVE_BALL, DEFENSIVE_NEUTRAL, NEUTRAL, OFFENSIVE_NEUTRAL, WE_HAVE_BALL };

}  // namespace rtt::ai::analysis

#endif  // ROBOTEAM_AI_ANALYSISREPORT_H
