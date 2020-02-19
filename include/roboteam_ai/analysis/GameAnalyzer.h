#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

#include "world/BallPossession.h"
#include "AnalysisReport.h"
#include "gtest/gtest_prod.h"
#include "world_new/views/RobotView.hpp"
#include "world_new/views/WorldDataView.hpp"

namespace rtt::ai::analysis {
namespace v = world_new::view;

class GameAnalyzer {
 public:
    static std::vector<std::pair<v::RobotView, RobotDanger>> getRobotsSortedOnDanger(const Field &field, bool ourTeam, v::WorldDataView world);
    static BallPossession convertPossession(rtt::ai::BallPossession::Possession possession);
    static double getTeamDistanceToGoalAvg(const Field &field, bool ourGoal, std::vector<v::RobotView> robots);
    static RobotDanger evaluateRobotDangerScore(const Field &field, v::RobotView robot, bool ourTeam, , v::WorldDataView world);
    static std::vector<std::pair<int, double>> getRobotsToPassTo(v::RobotView robot, bool ourTeam, v::WorldDataView world);
    static double shortestDistToEnemyRobot(v::RobotView robot, bool ourTeam, v::WorldDataView world);
};

}  // namespace rtt::ai::analysis
#endif  // ROBOTEAM_AI_GAMEANALYZER_H
