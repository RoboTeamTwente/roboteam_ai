//
// Created by mrlukasbos on 19-2-19.
//

#include "GameAnalyzer.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace analysis {

playStyle GameAnalyzer::getRecommendedPlayStyle(bool ourTeam) {

    /*
     * If they are close
     * and have an immediate shot at goal
     */
    return DEFEND_WITH_ALL;
}


double GameAnalyzer::getBallPossessionEstimate(bool ourTeam) {
    double ballPossessionEstimate = 0.0;

    // booleans that matter
    bool teamHasBall = World::teamHasBall(ourTeam);
    bool teamHasShotAtGoal = true;
    bool teamHasAttackerAimedAtGoal = true;

    // doubles that matter
    double teamDistanceToGoal = 0;

    // integers that matter
    int amountOfOurRobots = 0;
    int amountOfTheirRobots = 0;



    return 0;
}

/// Get the average of the distances of robots to their opponents goal
double GameAnalyzer::getTeamDistanceToGoalAvg(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getDistanceToGoal(ourTeam, robot.pos);
    }
    return (total / robots.size());
}

/// return the attackers of a given team sorted on their vision on their opponents goal
std::vector<std::pair<roboteam_msgs::WorldRobot, double>> GameAnalyzer::getAttackersSortedOnGoalVision(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> robotsWithVisibilities;

    for (auto robot : robots) {
        robotsWithVisibilities.emplace_back(robot, Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos));
    }

    // sort on goal visibility
    std::sort(robotsWithVisibilities.begin(), robotsWithVisibilities.end(),
            [](std::pair<roboteam_msgs::WorldRobot, double> a, std::pair<roboteam_msgs::WorldRobot, double> b){
       return  a.second < b.second;
    });

    return robotsWithVisibilities;
}

/// return the average goal vision of a given team towards their opponents goal
double GameAnalyzer::getTeamGoalVisionAvg(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);
    }
    return (total/robots.size());
}

} // analysis
} // ai
} // rtt