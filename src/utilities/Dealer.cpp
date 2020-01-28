#include <roboteam_utils/Hungarian.h>
#include "include/roboteam_ai/utilities/Dealer.h"

namespace rtt::ai {

// Dealer constructor
Dealer::Dealer(v::WorldDataView world, world::Field * field)
    : world(world), field(field) { }

// DealerFlag constructor
Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority)
    : title(std::move(title)), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(std::vector<v::RobotView> allRobots, const FlagMap &flagMap) {
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
    std::unordered_map<std::string, v::RobotView> result;
    for (int i = 0; i < roleNames.size(); i++) {
        auto robot = world.getRobotForId(assignment[i]);
        if (robot) {
            result.insert({roleNames[i], robot.value()});
        } else {
            std::cerr << "[Dealer] A robot was assigned but it got removed from world!" << std::endl;
        }
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
std::vector<vector<double>> Dealer::getScoreMatrix(std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap) {
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

double Dealer::getScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = getFactorForPriority(flag);
    return factor * getDefaultFlagScores(robot, flag);
}

double Dealer::getFactorForPriority(const Dealer::DealerFlag &flag) {
    switch (flag.priority) {
        case DealerFlagPriority::LOW_PRIORITY: return 1.0;
        case DealerFlagPriority::MEDIUM_PRIORITY: return 2.0;
        case DealerFlagPriority::HIGH_PRIORITY: return 3.0;
    }
}

// TODO 'invert' the matrix scores
double Dealer::getDefaultFlagScores(const v::RobotView &robot, const Dealer::DealerFlag &flag) {
    switch (flag.title) {
        case DealerFlagTitle::CLOSE_TO_THEIR_GOAL: {
            return field->getDistanceToGoal(false, robot->getPos());
        }
        case DealerFlagTitle::CLOSE_TO_OUR_GOAL: {
            return field->getDistanceToGoal(true, robot->getPos());
        }
        case DealerFlagTitle::CLOSE_TO_BALL: {
            auto ball = world.getBall();
            if (!ball) return 0.0;
            return robot->getPos().dist(ball.value()->getPos());
        }
        case DealerFlagTitle::HAS_WORKING_BALL_SENSOR: {
            return robot->isWorkingBallSensor() ? 1.0 : 0.0;
        }
        case DealerFlagTitle::ROBOT_TYPE_50W: {
            return robot->getRobotType() == world_new::robot::RobotType::FIFTY_WATT ? 1.0 : 0.0;
        }
        case DealerFlagTitle::ROBOT_TYPE_30W: {
            return robot->getRobotType() == world_new::robot::RobotType::THIRTY_WATT ? 1.0 : 0.0;
        }
        default: {
            std::cerr << "[Dealer] Unhandled dealerflag!" << endl;
            return 0;
        }
    }
}

} // rtt::ai