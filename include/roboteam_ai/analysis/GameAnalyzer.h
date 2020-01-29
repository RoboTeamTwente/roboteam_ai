//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

#include "AnalysisReport.h"
#include "gtest/gtest_prod.h"
#include "world/BallPossession.h"
#include "world/World.h"
#include "world/WorldData.h"

namespace rtt::ai::analysis {

class GameAnalyzer {
    FRIEND_TEST(GameAnalyzerTest, it_works);

   public:
    // It's a singleton; don't copy it.
    GameAnalyzer(const GameAnalyzer &) = delete;

    void operator=(const GameAnalyzer &) = delete;

    static GameAnalyzer &getInstance();

    void start(int iterationsPerSecond = Constants::GAME_ANALYSIS_TICK_RATE());
    void stop();

    std::shared_ptr<AnalysisReport> getMostRecentReport();
    std::shared_ptr<AnalysisReport> generateReportNow(const Field &field);

   private:
    using WorldData = world::WorldData;
    using RobotPtr = world::World::RobotPtr;
    using BallPtr = world::World::BallPtr;

    GameAnalyzer();

    // Threading
    std::thread thread;
    std::mutex mutex;
    volatile bool running;
    volatile bool stopping;
    void loop(unsigned delayMillis);

    std::shared_ptr<AnalysisReport> mostRecentReport;

    std::vector<std::pair<RobotPtr, RobotDanger>> getRobotsSortedOnDanger(const Field &field, bool ourTeam);
    BallPossession convertPossession(rtt::ai::BallPossession::Possession possession);
    double getTeamDistanceToGoalAvg(const Field &field, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    RobotDanger evaluateRobotDangerScore(const Field &field, RobotPtr robot, bool ourTeam);;

    std::vector<std::pair<int, double>> getRobotsToPassTo(RobotPtr robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    double shortestDistToEnemyRobot(RobotPtr robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    bool isClosingInToGoal(const Field &field, RobotPtr robot, bool ourTeam);
};

}  // namespace rtt::ai::analysis
#endif  // ROBOTEAM_AI_GAMEANALYZER_H
