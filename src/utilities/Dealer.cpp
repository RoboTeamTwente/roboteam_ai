
/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */

// TODO Fix issue where roles get redistributed whilst robots are already in position
// This issue occurs when there are multiple roles classes (defender+midfielder) that have the same priority
// 01-24-2022 emiel : Not sure if these are still a thing ^^^^^ Who wrote these?

#include "utilities/Dealer.h"

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Print.h>
#include <stdio.h>

#include <iterator>

#include "interface/api/Output.h"
#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai {

Dealer::Dealer(v::WorldDataView world, rtt_world::Field *field) : world(world), field(field) {}

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority) : title(title), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(std::vector<v::RobotView> robots, FlagMap role_to_flags,
                                                                 const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    // Return variable
    std::unordered_map<std::string, v::RobotView> role_assignment;

    /** Made up variables for the example given throughout this function. 6 robots and 4 roles to distribute.
     * robots          : [ RobotView id=0, RobotView id=2, RobotView id=5, RobotView id=3, RobotView id=9, RobotView id=4 ]
     * role_to_flags   : "keeper":REQUIRED,id=5, "Defender1":LOW_PRIORITY, "Attacker1":HIGH_PRIORITY,"Attacker2":HIGH_PRIORITY
     * role_assignment : empty
     */

    // Assign robots to roles that have fixed ids. In the example, the keeper requires id 5. During a game, the keeper id can not be changed. See the SSl rules.
    distributeFixedIds(robots, role_to_flags, role_assignment);

    /** Variables after the fixed ids have been distributed. Keeper has been assigned to robot 5, and 'robots' and 'role_to_flags' are pruned
     * robots          : [ RobotView id=0, RobotView id=2, RobotView id=3, RobotView id=9, RobotView id=4 ]
     * role_to_flags   : "Defender1":LOW_PRIORITY, "Attacker1":HIGH_PRIORITY,"Attacker2":HIGH_PRIORITY
     * role_assignment : "keeper":RobotView id=5,
     */

    // Calculate the cost matrix for each role and each robot
    std::vector<std::vector<double>> cost_matrix = getScoreMatrix(robots, role_to_flags, stpInfoMap);

    // Put all role names in a vector, for convenience
    std::vector<std::string> role_names;  // Holds all role names
    role_names.reserve(role_to_flags.size());
    for (auto const &imap : role_to_flags) role_names.push_back(imap.first);  // Fill with rolenames, e.g. KEEPER, DEFENDER_1, etc, etc

    // Create mappings between the rows and column of the cost matrix and the roles and robots. These will be modified during distribution
    std::vector<int> row_to_role(role_to_flags.size());              // maps a row to the original role
    std::vector<int> col_to_robot(robots.size());                    // maps a column to the original robot
    std::iota(std::begin(row_to_role), std::end(row_to_role), 0);    // Fill with 0 .. nRoles
    std::iota(std::begin(col_to_robot), std::end(col_to_robot), 0);  // Fill with 0 .. nRobots

    /** For the given example
     * role_names   : ["Defender1", "Attacker1", "Attacker2"]
     * row_to_role  : [0, 1, 2]
     * col_to_robot : [0, 1, 2 ,3, 4]
     *
     * cost_matrix[role (row)][robot (column)] containing the cost for each robot to be assigned a specific role.
     *                                  0    1    2    3    4  ←  col_to_robot
     * LOW_PRIORITY  Defender1   0   0.35 2.66 1.86 1.67 0.18
     * HIGH_PRIORITY Attacker1   1   0.19 2.32 2.78 1.02 1.31
     * HIGH_PRIORITY Attacker2   2   1.67 2.18 0.54 2.37 2.62
     *                           ↑
     *                      row_to_role
     */

    // Loop through the role priorities from high to low : KEEPER, REQUIRED, HIGH_PRIORITY, MEDIUM_PRIORITY, LOW_PRIORITY
    for (const auto current_priority : PriorityOrder) {
        /** Example : current_priority = HIGH_PRIORITY */

        // Uncomment the following line to print the current cost matrix. Useful for debugging
        // printCostMatrix(cost_matrix, role_names, robots, role_to_flags, row_to_role, col_to_robot);

        // Create a new cost matrix which will hold the cost_matrix rows for the priority we're currently at
        std::vector<std::vector<double>> cost_matrix_for_priority;
        cost_matrix_for_priority.reserve(cost_matrix.size());  // Reserve rows
        // This vector will hold the indices of the cost_matrix rows that have the current priority
        std::vector<int> row_indices;
        row_indices.reserve(cost_matrix.size());  // Reserve number of rows

        /* Go over each role (row) in cost_matrix and check if the role has the current priority. If so, copy row into cost_matrix_for_priority */
        for (int row = cost_matrix.size() - 1; 0 <= row; row--) {
            // Check if the role corresponding to the cost matrix row has the current priority
            if (role_to_flags.at(role_names[row_to_role[row]]).priority == current_priority) {
                row_indices.push_back(row);
                // Copy the row from the cost_matrix into the cost_matrix_for_priority.
                cost_matrix_for_priority.push_back(cost_matrix[row]);
                // NOTE There is a small performance gain to be made here by using push_back(std::move(cost_matrix[row])). The matrix must then be traversed bottom to top.
                // NOTE However, this leaves the row in the cost matrix in an unspecified state. Accidentally accessing it would give undefined behaviour and possible segfaults
                // NOTE Useful resource on push_back() vs push_back(move()) https://stackoverflow.com/questions/11572669/move-with-vectorpush-back
            }
        }

        /** For the given example
         * The rows for HIGH_PRIORITY (1 and 2) have been copied from cost_matrix into cost_matrix_for_priority
         * row_indices : [1, 2]
         * cost_matrix_for_priority :
         *     0.19 2.32 2.78 1.02 1.31
         *     1.67 2.18 0.54 2.37 2.62
         */

        // If there are no roles with the current priority, continue to the next priority
        if (row_indices.empty()) continue;

        /* Assign roles to robots using the cost matrix and the Hungarian algorithm.
         * If there are more roles than robots, not all roles can be assigned.
         * In that case, the value -1 is used to indicate that a role is not assigned */
        std::vector<int> assignments;
        rtt::Hungarian::Solve(cost_matrix_for_priority, assignments);

        /* For each role (row) with the current priority, connect it to a robot using the calculated assignments */
        for (std::size_t i = 0; i < row_indices.size(); i++) {
            if (0 <= assignments[i]) {
                std::string role_name = role_names[row_to_role[row_indices[i]]];
                v::RobotView robot = robots[col_to_robot[assignments[i]]];
                role_assignment.insert({role_name, robot});
            }
        }

        /** For the given example, looking at cost_matrix_for_priority, it's optimal to assign row 0 to column 0 and row 1 to column 2, for a total cost of 0.73
         * assignments : [0, 2]
         * The variable col_to_robot is used to map the columns 0 and 2 to the robots with id 0 and 3. e.g. robots[col_to_robot[2]].id = 3
         * role_assignment : "keeper":RobotView id=5, "Attacker1":RobotView id=0, "Attacker2":RobotView id=3
         */

        /* == Do not use row_indices and assignments below this line. They lose their meaning when sorted == */

        /* Remove each assigned role (row) from both cost_matrix and row_to_role vector */
        // Sort the row_indices in descending order. This is done so that the corresponding cost_matrix rows are deleted from bottom to top
        std::sort(row_indices.begin(), row_indices.end(), std::greater());
        for (int row : row_indices) {
            cost_matrix.erase(cost_matrix.begin() + row);
            row_to_role.erase(row_to_role.begin() + row);
        }

        /* Remove each robot (column) from both the cost_matrix and col_to_robot */
        // Sort the assignments in descending order. This is done so that the corresponding cost_matrix columns are deleted from bottom to top
        std::sort(assignments.begin(), assignments.end(), std::greater());
        for (int col : assignments) {
            // As stated before, -1 indicates that no robot has been assigned. Therefore, no robot has to be removed.
            if (0 <= col) {
                for (std::vector<double> &row : cost_matrix) row.erase(row.begin() + col);
                col_to_robot.erase(col_to_robot.begin() + col);
            }
        }

        /** For the given example. The distributed roles and robots have to be removed from the cost matrix. This is done using the variables 'row_indices' and 'assignment'
         * row_indices = [1, 2], assignment = [0, 2], so rows 1 and 2 and columns 0 and 2 will be removed. The variables row_to_role and col_to_robot have to be updated as
         * well to preserve the mapping. These are the variables after updating:
         * row_to_role  : [0]
         * col_to_robot : [1, 3, 4]
         *
         * cost_matrix[role (row)][robot (column)] containing the cost for each robot to be assigned a specific role.
         *                                 1    3    4  ←  col_to_robot
         * LOW_PRIORITY  Defender1   0  2.66 1.67 0.18
         *                           ↑
         *                      row_to_role
         */

        /* if no more roles    or no more robots      , then stop */
        if (row_to_role.empty() or col_to_robot.empty()) break;
    }

    setGameStateRoleIds(role_assignment);
    return role_assignment;
}

void Dealer::distributeFixedIds(std::vector<v::RobotView> &robots, FlagMap &flagMap, std::unordered_map<std::string, v::RobotView> &assignments) {
    auto role = flagMap.begin();
    while (role != flagMap.end()) {
        int required_id = role->second.forcedID;
        if (required_id < 0) {
            role++;
            continue;
        }

        /* Find the required id in the given list of robots */
        // https://www.techiedelight.com/remove-entries-map-iterating-cpp/
        bool robot_found = false;                                            // Stores if the robot has been found, for logging purposes
        for (std::size_t i_robot = 0; i_robot < robots.size(); i_robot++) {  // Iterate over each robot
            // Found the robot with the right id
            if (robots[i_robot]->getId() == required_id) {           // If the correct robot has been found
                robot_found = true;                                  //
                assignments.insert({role->first, robots[i_robot]});  //     Assign the robot to the role
                robots.erase(robots.begin() + i_robot);              //     Remove the robot from the list of robot
                role = flagMap.erase(role);                          //     Remove the role from the map of roles
                break;
            }
        }

        if (!robot_found) RTT_ERROR("Could not find robot with required id ", required_id, " for role ", role->first, ". This forced assignment will be ignored.");
        role++;
    }
}

// Populate a matrix with scores
std::vector<std::vector<double>> Dealer::getScoreMatrix(const std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap,
                                                        const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    std::vector<std::vector<double>> cost_matrix;
    cost_matrix.reserve(flagMap.size());

    // Loop through all roles that are in the dealerFlags map
    for (auto const &[roleName, dealerFlags] : flagMap) {
        std::vector<double> robot_costs_for_role;
        robot_costs_for_role.reserve(allRobots.size());
        // Calculate the score for each robot for a role; the row
        for (auto robot : allRobots) {
            double robotDistanceScore{};
            if (stpInfoMap.find(roleName) != stpInfoMap.end()) {
                // The shorter the distance, the lower the distance score
                robotDistanceScore = getRobotScoreForDistance(stpInfoMap.find(roleName)->second, robot);
            }
            // The better the flags, the lower the score
            auto robotScore = getRobotScoreForRole(dealerFlags.flags, robot);
            // Simple normalizer. DistanceScore has weight 1, the other factors can have various weights.
            double robotRoleScore = (robotDistanceScore + robotScore.sumScore) / (robotScore.sumWeights + 1);  // the +1 is the distanceScore weight
            robot_costs_for_role.push_back(robotRoleScore);
        }
        cost_matrix.push_back(std::move(robot_costs_for_role));
    }
    return cost_matrix;
}

// Calculate the score for all flags for a role for one robot
Dealer::RobotRoleScore Dealer::getRobotScoreForRole(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot) {
    double robotScore = 0;
    double sumWeights = 0;
    for (auto flag : dealerFlags) {
        FlagScore ScoreForFlag = getRobotScoreForFlag(robot, flag);
        robotScore += ScoreForFlag.score;
        sumWeights += ScoreForFlag.weight;
    }
    return {robotScore, sumWeights};  // [score,sum of weights]
}

// Get the score of one flag for a role for one robot
Dealer::FlagScore Dealer::getRobotScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = getWeightForPriority(flag.priority);
    return {factor * getDefaultFlagScores(robot, flag), factor};  // [score,weight]
}

// Get the distance score for a robot to a position when there is a position that role needs to go to
double Dealer::getRobotScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot) {
    double distance{};

    std::optional<Vector2> target_position;
    // Search for position in getEnemyRobot, getPositionToDefend, and getPositionToMoveTo
    if (stpInfo.getEnemyRobot().has_value()) target_position = stpInfo.getEnemyRobot().value()->getPos();
    if (stpInfo.getPositionToDefend().has_value()) target_position = stpInfo.getPositionToDefend().value();
    if (stpInfo.getPositionToMoveTo().has_value()) target_position = stpInfo.getPositionToMoveTo().value();
    // If robot is keeper, set distance to self. Basically 0
    if (stpInfo.getRoleName() == "keeper" && robot->getId() == GameStateManager::getCurrentGameState().keeperId) target_position = robot->getPos();

    // No target found to move to
    if (!target_position) return 0;

    // Target found. Calculate distance
    distance = robot->getPos().dist(*target_position);

    return costForDistance(distance, field->getFieldWidth(), field->getFieldLength());
}

// TODO these values need to be tuned.
double Dealer::getWeightForPriority(const DealerFlagPriority &flagPriority) {
    switch (flagPriority) {
        case DealerFlagPriority::LOW_PRIORITY:
            return 0.5;
        case DealerFlagPriority::MEDIUM_PRIORITY:
            return 1;
        case DealerFlagPriority::HIGH_PRIORITY:
            return 5;
        case DealerFlagPriority::REQUIRED:
            return 100;
        case DealerFlagPriority::KEEPER:
            return 1000;
        default:
            RTT_WARNING("Unhandled dealerflag!")
            return 0;
    }
}

// TODO these values need to be tuned.
double Dealer::getDefaultFlagScores(const v::RobotView &robot, const Dealer::DealerFlag &flag) {
    auto fieldWidth = field->getFieldWidth();
    auto fieldLength = field->getFieldLength();
    switch (flag.title) {
        case DealerFlagTitle::CLOSE_TO_THEIR_GOAL:
            return costForDistance(FieldComputations::getDistanceToGoal(*field, false, robot->getPos()), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_OUR_GOAL:
            return costForDistance(FieldComputations::getDistanceToGoal(*field, true, robot->getPos()), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_BALL:
            return costForDistance(robot->getDistanceToBall(), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_POSITIONING:
            return costForProperty(true);
        case DealerFlagTitle::WITH_WORKING_BALL_SENSOR:
            return costForProperty(robot->isWorkingBallSensor());
        case DealerFlagTitle::WITH_WORKING_DRIBBLER:
            return costForProperty(robot->isWorkingDribbler());
        case DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT: {
            // get distance to line between ball and goal
            // TODO this method can be improved by choosing a better line for the interception.
            LineSegment lineSegment = {world.getBall()->get()->position, field->getOurGoalCenter()};
            return lineSegment.distanceToLine(robot->getPos());
        }
        case DealerFlagTitle::KEEPER:
            return costForProperty(robot->getId() == GameStateManager::getCurrentGameState().keeperId);
        case DealerFlagTitle::CLOSEST_TO_BALL:
            return costForProperty(robot->getId() == world.getRobotClosestToBall(rtt::world::us)->get()->getId());
        case DealerFlagTitle::NOT_IMPORTANT:
            return 0;
        case DealerFlagTitle::CAN_DETECT_BALL: {
            bool hasWorkingBallSensor = Constants::ROBOT_HAS_WORKING_BALL_SENSOR(robot->getId());
            bool hasDribblerEncoder = Constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(robot->getId());
            return costForProperty(hasWorkingBallSensor || hasDribblerEncoder);
        }
        case DealerFlagTitle::CAN_KICK_BALL: {
            bool hasWorkingKicker = Constants::ROBOT_HAS_KICKER(robot->getId());
            return costForProperty(hasWorkingKicker);
        }
        default: {
            RTT_WARNING("Unhandled dealerflag!")
            return 0;
        }
    }
}

void Dealer::setGameStateRoleIds(std::unordered_map<std::string, v::RobotView> output) {
    if (output.find("keeper") != output.end()) {
        interface::Output::setKeeperId(output.find("keeper")->second->getId());
    }
    if (output.find("ball_placer") != output.end()) {
        interface::Output::setBallPlacerId(output.find("ball_placer")->second->getId());
    }
}

// Calculate the cost for distance. The further away the target, the higher the cost for that distance.
double Dealer::costForDistance(double distance, double fieldWidth, double fieldHeight) {
    auto fieldDiagonalLength = sqrt(pow(fieldWidth, 2.0) + pow(fieldHeight, 2.0));
    return distance / fieldDiagonalLength;
}

double Dealer::costForProperty(bool property) { return property ? 0.0 : 1.0; }

}  // namespace rtt::ai