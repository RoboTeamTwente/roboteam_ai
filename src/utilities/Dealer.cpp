
/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */

// TODO Fix issue where roles get redistributed whilst robots are already in position
/// This issue occurs when there are multiple roles classes (defender+midfielder) that have the same priority

#include "utilities/Dealer.h"

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Print.h>

#include <iterator>

#include "interface/api/Output.h"
#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

#include <stdio.h>
#include <chrono>

namespace rtt::ai {


void Dealer::printCostMatrix(const std::vector<std::vector<double>> &cost_matrix,  const std::vector<std::string> &role_names, const std::vector<v::RobotView> &robots,
                             const std::vector<int> &row_to_role, const std::vector<int> &col_to_robot){
    std::cout << "[printCostMatrix] " << row_to_role.size() << " roles and " << col_to_robot.size() << " robots" << std::endl;
    for(int row = 0; row < cost_matrix.size(); row++){
        std::cout << "  " << std::setw(15) << role_names[row_to_role[row]];
        const std::vector<double> &scores_per_robot = cost_matrix[row];
        for(int i_robot = 0; i_robot < scores_per_robot.size(); i_robot++)
            std::cout << std::right << std::setw(3) << robots[col_to_robot[i_robot]]->getId() << "=" << std::left << std::setw(7) << std::setprecision(4) << scores_per_robot[i_robot];
        std::cout << std::endl;
    }
}

Dealer::Dealer(v::WorldDataView world, rtt_world::Field *field) : world(world), field(field) { }

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority) : title(title), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(std::vector<v::RobotView> robots, FlagMap role_map, const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    // https://levelup.gitconnected.com/8-ways-to-measure-execution-time-in-c-c-48634458d0f9
    auto time_start = std::chrono::high_resolution_clock::now();
    int n_robots = robots.size();

    std::unordered_map<std::string, v::RobotView> role_assignment;

    // Remove all forcedID's before continuing computations
    distribute_forcedIDs(robots, role_map, role_assignment);

    /** cost_matrix[role (row)][robot (column)] containing the cost for each robot to take up a specific role.
     *                    0    1    2    3    4  ←  col_to_robot
     * Attacker1   0   0.19 2.32 2.78 1.02 1.31
     * Attacker2   1   1.67 2.18 0.54 2.37 2.62
     * Defender1   2   0.35 2.66 1.86 1.67 0.18
     *             ↑
     *        row_to_role
     **/
    std::vector<std::vector<double>> cost_matrix = getScoreMatrix(robots, role_map, stpInfoMap);

    // Make index of roles and ID to keep track which are the original indexes of each and get role_names
    std::vector<int> row_to_role(role_map.size());                               // [0, 1, 2]
    std::vector<int> col_to_robot(robots.size());                                // [0, 1, 2, 3, 4]
    std::iota(std::begin(row_to_role), std::end(row_to_role), 0);    // Fill with 0 .. nRoles
    std::iota(std::begin(col_to_robot), std::end(col_to_robot), 0);  // Fill with 0 .. nRobots
    std::vector<std::string> role_names;                                         // [Attacker1, Attacker2, Defender1]
    role_names.reserve(role_map.size());
    for (auto const &imap : role_map) role_names.push_back(imap.first);           // Fill with rolenames, e.g. KEEPER, DEFENDER_1, etc, etc

    // Loop through the role priorities from high to low : KEEPER, REQUIRED, HIGH_PRIORITY, MEDIUM_PRIORITY, LOW_PRIORITY
    for (const auto current_priority : PriorityOrder) {

        //        std::cout << "\nCurrent priority " << priorityToString(current_priority) << " nRoles=" << cost_matrix.size() << std::endl;
        //        printCostMatrix(cost_matrix, role_names, robots, row_to_role, col_to_robot);

        // Create a new cost matrix which will hold the cost_matrix rows for the priority we're currently at
        std::vector<std::vector<double>> cost_matrix_for_priority;
        cost_matrix_for_priority.reserve( cost_matrix.size() );     // Reserve rows
        // This vector will hold the indices of the cost_matrix rows that have the current priority
        std::vector<int> row_indices;
        row_indices.reserve( cost_matrix.size() );  // Reserve number of rows

        /* Go over each role (row) in cost_matrix and check if the role has the current priority. If so, copy row into cost_matrix_for_priority */
        for(int row = cost_matrix.size() - 1; 0 <= row; row-- ){
            // Check if the role corresponding to the cost matrix row has the current priority
            if(role_map.at( role_names[row_to_role[row]] ).priority == current_priority) {
                row_indices.push_back(row);
                // Copy the row from the cost_matrix into the cost_matrix_for_priority.
                cost_matrix_for_priority.push_back(cost_matrix[row]);
                // NOTE There is a small performance gain to be made here by using push_back(cost_matrix[row]). The matrix must be traversed bottom to top.
                // NOTE However, this leaves the row in the cost matrix in an unspecified state. Accidentally accessing it would give undefined behaviour and possible segfaults
                // NOTE Useful resource on push_back() vs push_back(move()) https://stackoverflow.com/questions/11572669/move-with-vectorpush-back
            }
        }

        // If there are no roles with the current priority, continue to the next priority
        if(row_indices.empty()) continue;

        /* Assign roles to robots using the cost matrix and the Hungarian algorithm.
         * If there are more roles than robots, not all roles can be assigned.
         * In that case, the value -1 is used to indicate that a role is not assigned */
        std::vector<int> assignments;
        rtt::Hungarian::Solve(cost_matrix_for_priority, assignments);

        //        for(int i = 0; i < row_indices.size(); i++)
        //            if(0 <= assignments[i])
        //                std::cout << "Row " << row_indices[i] << " (" << role_names[row_to_role[row_indices[i]]] << ") given to robot " << robots[col_to_robot[assignments[i]]]->getId() << std::endl;

        /* For each role (row) with the current priority, connect it to a robot using the calculated assignments */
        for(int i = 0; i < row_indices.size(); i++){
            if(0 <= assignments[i]) {
                std::string role_name = role_names[row_to_role[row_indices[i]]];
                v::RobotView robot = robots[col_to_robot[assignments[i]]];
                role_assignment.insert({role_name, robot});
            }
        }

        /* == Do not use row_indices and assignments below this line. They lose their meaning when sorted == */

        /* Remove each assigned role (row) from both cost_matrix and row_to_role vector */
        // Sort the row_indices in descending order. This is done so that the corresponding cost_matrix rows are deleted from bottom to top
        std::sort(row_indices.begin(), row_indices.end(), std::greater());
        for(int row : row_indices){
            cost_matrix.erase(cost_matrix.begin() + row);
            row_to_role.erase(row_to_role.begin() + row);
        }

        /* Remove each robot (column) from both the cost_matrix and col_to_robot */
        // Sort the assignments in descending order. This is done so that the corresponding cost_matrix columns are deleted from bottom to top
        std::sort(assignments.begin(), assignments.end(), std::greater());
        for(int col : assignments){
            // As stated before, -1 indicates that no robot has been assigned. Therefore, no robot has to be removed.
            if(0 <= col) {
                for (std::vector<double> &row : cost_matrix) row.erase(row.begin() + col);
                col_to_robot.erase(col_to_robot.begin() + col);
            }
        }

        /* if no more roles    or no more robots      , then stop */
        if(row_to_role.empty() or col_to_robot.empty()) break;

    }

    //    for (auto& [role_name, robot]: role_assignment)
    //        std::cout << "role_assignment: Role " << role_name << " assigned to robot " << robot->getId() << std::endl;

    auto time_stop = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(time_stop - time_start);
    auto milliseconds = std::chrono::duration_cast< std::chrono::microseconds >( elapsed );

    RTT_INFO("Distributed ", role_assignment.size(), " roles to ", n_robots, " robots in ", milliseconds.count(), " microseconds");

    setGameStateRoleIds(role_assignment);
    return role_assignment;
}

void Dealer::distribute_forcedIDs(std::vector<v::RobotView> &robots, FlagMap &flagMap, std::unordered_map<std::string, v::RobotView> &assignments) {

    auto role = flagMap.begin();
    while(role != flagMap.end()){
        int required_id = role->second.forcedID;
        if(required_id == -1){
            role++;
            continue;
        }

        /* Find the required id in the given list of robots */
        // https://www.techiedelight.com/remove-entries-map-iterating-cpp/
        bool robot_found = false;                                   // Stores if the robot has been found, for logging purposes
        for(int i_robot = 0; i_robot < robots.size(); i_robot++){   // Iterate over each robot
            // Found the robot with the right id
            if(robots[i_robot]->getId() == required_id){                // If the correct robot has been found
                robot_found = true;                                     //
                assignments.insert({role->first, robots[i_robot]});  //     Assign the robot to the role
                robots.erase(robots.begin() + i_robot);                 //     Remove the robot from the list of robot
                role = flagMap.erase(role);                             //     Remove the role from the map of roles
                break;
            }
        }

        if(!robot_found)
            RTT_ERROR("Could not find robot with required id ", required_id, " for role ", role->first, ". This forced assignment will be ignored.");
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
            double robotRoleScore = (robotDistanceScore + robotScore.sumScore) / (robotScore.sumWeights + 1); // the +1 is the distanceScore weight
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
    if(stpInfo.getRoleName() == "keeper" && robot->getId() == GameStateManager::getCurrentGameState().keeperId)
        target_position = robot->getPos();

    // No target found to move to
    if(!target_position) return 0;

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
            LineSegment lineSegment = {world.getBall()->get()->getPos(), field->getOurGoalCenter()};
            return lineSegment.distanceToLine(robot->getPos());
        }
        case DealerFlagTitle::KEEPER:
            return costForProperty(robot->getId() == GameStateManager::getCurrentGameState().keeperId);
        case DealerFlagTitle::CLOSEST_TO_BALL:
            return costForProperty(robot->getId() == world.getRobotClosestToBall(rtt::world::us)->get()->getId());
    }
    RTT_WARNING("Unhandled dealerflag!")
    return 0;
}

void Dealer::setGameStateRoleIds(std::unordered_map<std::string, v::RobotView> output) {
    std::cout << "[setGameStateRoleIds]" << std::endl;
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

std::string Dealer::priorityToString(DealerFlagPriority priority){
    switch (priority) {
        case DealerFlagPriority::LOW_PRIORITY:
            return "LOW_PRIORITY";
        case DealerFlagPriority::MEDIUM_PRIORITY:
            return "MEDIUM_PRIORITY";
        case DealerFlagPriority::HIGH_PRIORITY:
            return "HIGH_PRIORITY";
        case DealerFlagPriority::REQUIRED:
            return "REQUIRED";
        case DealerFlagPriority::KEEPER:
            return "KEEPER";
        default:
            return "UNKNOWN PRIORITY";
    }
}

}  // namespace rtt::ai