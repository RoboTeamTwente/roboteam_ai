//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

#include "AnalysisReport.h"
#include "../world/WorldData.h"
#include "../world/World.h"
#include "../world/BallPossession.h"
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
        using WorldData = world::WorldData;
        using Robot = world::Robot;
        using Ball = world::Ball;

    GameAnalyzer();

    // Threading
    std::thread thread;
    std::mutex mutex;
    volatile bool running;
    volatile bool stopping;
    void loop(unsigned delayMillis);

    std::shared_ptr<AnalysisReport> mostRecentReport;

    std::vector<std::pair<Robot, RobotDanger>> getRobotsSortedOnDanger(bool ourTeam);
    BallPossession getBallPossessionEstimate();
    BallPossession convertPossession(rtt::ai::BallPossession::Possession possession);
    double getTeamDistanceToGoalAvg(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    double getTeamGoalVisionAvg(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    RobotDanger evaluateRobotDangerScore(Robot robot, bool ourTeam);
    std::vector<std::pair<Robot, double>> getAttackersSortedOnGoalVision(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());

    std::vector<std::pair<int, double>> getRobotsToPassTo(Robot robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    double shortestDistToEnemyRobot(Robot robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    bool isClosingInToGoal(Robot robot, bool ourTeam);
};


}
}
}
#endif //ROBOTEAM_AI_GAMEANALYZER_H
