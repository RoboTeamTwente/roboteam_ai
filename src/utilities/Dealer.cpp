#include <roboteam_utils/Hungarian.h>
#include "include/roboteam_ai/utilities/Dealer.h"

namespace rtt::ai {

// DealerFlag constructor
Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, bool important)
    : title(std::move(title)), important(important) {}

// Role constructor
Dealer::Role::Role(std::string roleName, int robot) : name(std::move(roleName)), robotId(robot) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, int> Dealer::distribute(std::vector<int> allRobots, const FlagMap &flagMap) {
    std::vector<std::vector<double>> scores = getScoreMatrix(allRobots, flagMap);
    std::vector<int> assignment;

    // solve the matrix and put the results in 'assignment'
    rtt::Hungarian::Solve(scores, assignment);

    // get a list of rolenames in order
    std::vector<std::string> roleNames;
    for (auto const& [roleName, dealerFlags] : flagMap) {
        roleNames.push_back(roleName);
    }

    /* assignments now has the robot ids at the role index, and is ordered according to the roleNames
     * for example: assignments[0] = 2 // robot_id
     * and roleNames[0] = role_1
     * --> we can therefore make a map of <rolename, robot_id>
     */
    std::unordered_map<std::string, int> result;
    for (int i = 0; i < roleNames.size(); i++) {
        result.insert({roleNames[i], assignment[i]});
    }
    return result;
}

/* Populate a matrix with scores, such that:
 * ---------------------------------
 *         robot_1 robot_2 robot_3
 * role_1     12       11     0
 * role_2     3        3      1
 * role_3     22       1      2
 * ---------------------------------
 */
// TODO 'invert' the matrix scores
std::vector<vector<double>> Dealer::getScoreMatrix(vector<int> &allRobots, const Dealer::FlagMap &flagMap) {
    vector<vector<double>> scores;
    for (int column = 0; column < allRobots.size(); column++) {
        auto robot = allRobots.at(column);
        int row = 0;
        for (auto const& [roleName, dealerFlags] : flagMap) {
            double robotScore = 0;
            for (auto flag : dealerFlags) {
                robotScore += getScoreForFlag(robot, flag);
            }
            scores[column][row] = robotScore;
            row++;
        }
    }
    return scores;
}

int Dealer::getScoreForFlag(int robotId, Dealer::DealerFlag flag) {

    switch (flag.title) {
        case DealerFlagTitle::CLOSE_TO_THEIR_GOAL: {
            return 0;
        }
        case DealerFlagTitle::CLOSE_TO_OUR_GOAL: {
            return 0;
        }
        case DealerFlagTitle::CLOSE_TO_BALL: {
            return 0;
        }
        default: {
            std::cerr << "[Dealer] Unhandled dealerflag!" << std::endl;
            return 0;
        }
    }
}


} // rtt::ai