//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

#include <roboteam_msgs/WorldRobot.h>
#include "AnalysisReport.h"

namespace rtt {
namespace ai {
namespace analysis {

class GameAnalyzer {
public:
    // It's a singleton; don't copy it.
    GameAnalyzer(const GameAnalyzer &) = delete;
    void operator=(const GameAnalyzer &) = delete;
    static GameAnalyzer &getInstance();

    void start(int iterationsPerSecond = Constants::GAME_ANALYSIS_TICK_RATE());
    void stop();

    std::shared_ptr<AnalysisReport> getMostRecentReport();
    std::shared_ptr<AnalysisReport> generateReportNow();

private:
    GameAnalyzer();

    // Threading
    std::thread thread;
    std::mutex mutex;
    volatile bool running;
    volatile bool stopping;
    void loop(unsigned delayMillis);

    std::shared_ptr<AnalysisReport> mostRecentReport;

    std::vector<std::pair<roboteam_msgs::WorldRobot, RobotDanger>> getRobotsSortedOnDanger(bool ourTeam);
    double getBallPossessionEstimate(bool ourTeam);
    playStyle getRecommendedPlayStyle();
    double getTeamDistanceToGoalAvg(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    double getTeamGoalVisionAvg(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    RobotDanger evaluateRobotDangerScore(roboteam_msgs::WorldRobot robot, bool ourTeam);
    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> getAttackersSortedOnGoalVision(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());

    std::vector<std::pair<int, double>> getRobotsToPassTo(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    double shortestDistToEnemyRobot(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    bool isClosingInToGoal(roboteam_msgs::WorldRobot robot, bool ourTeam);
};


}
}
}
#endif //ROBOTEAM_AI_GAMEANALYZER_H
