//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

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

class GameAnalyzer {
public:
    GameAnalyzer() = default;
    double getBallPossessionEstimate(bool ourTeam);
    playStyle getRecommendedPlayStyle(bool ourTeam);
private:
    double getTeamDistanceToGoalAvg(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world()); // get average of distances to goal
    double getTeamGoalVisionAvg(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    double evaluateRobotDangerScore(roboteam_msgs::WorldRobot robot, bool ourTeam);
    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> getAttackersSortedOnGoalVision(bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());

    double robotCanShootAtGoalCertainty(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    vector<pair<int, double>> getRobotsToPassTo(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    double shortestDistToEnemyRobot(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld = World::get_world());
    bool isClosingInToGoal(roboteam_msgs::WorldRobot robot, bool ourTeam);
};


}
}
}
#endif //ROBOTEAM_AI_GAMEANALYZER_H
