//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

#include "gtest/gtest_prod.h"
#include "AnalysisReport.h"
#include "world/WorldData.h"
#include "world/World.h"
#include "world/BallPossession.h"

namespace rtt {
namespace ai {
namespace analysis {

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
    std::shared_ptr<AnalysisReport> generateReportNow();

private:
        using WorldData = world::WorldData;
        using RobotPtr = world::World::RobotPtr;
        using BallPtr = world::World::BallPtr;

    GameAnalyzer();


    // Threading
    std::thread thread;
    std::mutex mutex;
    /**
     * There is no need for these to be volatile
     * The compiler will never optimize a variable
     * away if it really matters, maybe you meant
     * for these to be std::atomic, safe atomic
     * operations.
     * 
     * Volatile is often confused with threadsafe 
     * or even concurrent, whereas the only thing
     * it does, is make sure that the compiler does
     * not optimize your varable away
     */
    volatile bool running;
    volatile bool stopping;
    void loop(unsigned delayMillis);

    std::shared_ptr<AnalysisReport> mostRecentReport;

    std::vector<std::pair<RobotPtr, RobotDanger>> getRobotsSortedOnDanger(bool ourTeam);
    BallPossession convertPossession(rtt::ai::BallPossession::Possession possession);
    double getTeamDistanceToGoalAvg(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    double getTeamGoalVisionAvg(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    RobotDanger evaluateRobotDangerScore(RobotPtr robot, bool ourTeam);
    std::vector<std::pair<RobotPtr, double>> getAttackersSortedOnGoalVision(bool ourTeam, WorldData simulatedWorld = world::world->getWorld());

    std::vector<std::pair<int, double>> getRobotsToPassTo(RobotPtr robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    double shortestDistToEnemyRobot(RobotPtr robot, bool ourTeam, WorldData simulatedWorld = world::world->getWorld());
    bool isClosingInToGoal(RobotPtr robot, bool ourTeam);
};


}
}
}
#endif //ROBOTEAM_AI_GAMEANALYZER_H
