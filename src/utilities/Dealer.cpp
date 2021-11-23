// see Dealer.h
#include "utilities/Dealer.h"

namespace rtt::ai {
    using namespace std;
    using namespace stp;
    using namespace rtt::world;
    using namespace rtt::world::view;
    using namespace DealerSpecific;
    using FlagMap = unordered_map<string, Dealer::RoleInstruction>;
    using StpMap = unordered_map<string, StpInfo>;
    using AssignmentMap = unordered_map<string, RobotView>;
    using Matrix = vector<vector<int>>;

    Dealer::Dealer(WorldDataView world, Field *field) : m_world{world}, m_field{field} {}
    Dealer::~Dealer() = default;
    /// Explicit constructor for the Play inheritors to prioritizes role strategies/flags
    Dealer::FlagInstruction::FlagInstruction(Flag title, Priority level): title{title}, level{level}
    {}

    /**
     * Provides access to the private member functions and delegates the tasks from set-up to output
     * Distribution starts by performing basic checks for forcedIDs and equal numbers of roles and robots
     * After which, the costs matrix is calculated and passed to the Munkres Assignment Algorithm to identify the most cost effective distribution of roles to robots
     * Lastly, the returned vector from the assignment is translated into the output map
     * @param allRobots the vector storing the robots detected on the field
     * @param allRoles the map storing the flags and priorities of the roles set by the Play
     * @param allRoleInfo the map storing the last stored info for the roles set by the Play (like the target position or the last assigned robot)
     * @return an assignment map holding the roleNames and corresponding RobotViews
     * @note: allRobots and allRoles are copies as they are modified throughout run-time
     * @warning: This function assumes that allRobots is sorted by ascending robot IDs
     */
    AssignmentMap Dealer::distribute(vector<RobotView> allRobots, FlagMap allRoles, const StpMap &allRoleInfo) {
//        auto world{World::instance()};
//        std::optional<view::WorldDataView> previous = world.data->getHistoryWorld(1);
//        if(previous!=nullopt) m_world = previous.value();
    if(!allRoleInfo.empty()) {
        for (auto &role: allRoleInfo) {
            cout << role.first << " ";
        }
        cout << endl;
        for (auto &role: allRoleInfo) {
            if(role.second.getRobot()!=nullopt)cout << role.second.getRobot().value()->getId() << " ";
        }
        cout << endl;
    }

        AssignmentMap output(allRoles.size());
        // check for forcedIDs set in the Play
        auto forcedRobots{removeForcedIDs(allRobots, allRoles, output)};
        // ensure allRobots and allRoles are evenly sized
        if (allRobots.size() != allRoles.size()) {
            if (removeRoles(allRobots, allRoles) != 0) RTT_ERROR(
                    "Removing Roles Failed! Unable to match the number of robots and roles.")
        }
        // enable random access for all role names
        vector<string> roleNames;
        for (const auto &role: allRoles) roleNames.push_back(role.first);
        for (const auto &role: roleNames) cout << role << " ";
        cout << endl;

        // fill cost matrix and calculated optimal assignment with the help of the Hungarian Linear Assignment Algorithm (Munkres)
        auto costs{getCostMatrix(allRobots, allRoles, allRoleInfo)};
        auto laf{MunkresAssignment()};
        auto newAssignments{laf.runMunkres(costs)};

        // double check for error in the assignment output
        if (newAssignments.empty()) RTT_ERROR("Linear Assignment Failed! Unable to retrieve new assignment vector.")
        if (newAssignments.size() != allRobots.size()) RTT_ERROR("Linear Assignment Failed! Not all robots were assigned a role or vice-versa.")
        // store new assignments in the output map
        // the boost variable is unfortunately necessary, since the allRobots vector is index sensitive and removing forcedIDs impacts this
        auto boost{0};
        cout << "Previous: " << endl;
        for (int i{0}; i < newAssignments.size(); ++i) {
            if (newAssignments[i] < 0) RTT_ERROR("Linear Assignment Failed! Invalid robot ID found in new assignment vector.")
            if (!forcedRobots.empty() && i == forcedRobots.front()) {
                ++boost;
                forcedRobots.erase(forcedRobots.begin());
            }
            output.insert({roleNames[i], allRobots[newAssignments[i] + boost]});
            auto role = allRoleInfo.find(roleNames[i]);
            if(role != allRoleInfo.end() && role->second.getRobot().has_value()) cout << role->second.getRobot()->get()->getId() << " ";
            //allRobots[newAssignments[i] + boost]->newTargetPos(allRoleInfo.find(roleNames[i])->second.getPositionToMoveTo().value());

            //if (allRoleInfo.find(roleNames[i]) != allRoleInfo.end()) getCostForDistance(allRobots[newAssignments[i] + boost], allRoleInfo.find(roleNames[i])->second);
        }
        cout << endl;
        return output;
    }
//    AssignmentMap Dealer::distribute(vector<RobotView> allRobots, FlagMap allRoles, const StpMap &allRoleInfo) {
//        AssignmentMap output(allRoles.size());
//        for(const auto priority : PriorityOrder){
//            for(const auto role: allRoles){
//                if(role.second.priority == priority && allRoleInfo.contains(role.first)){
//                    if(role.first == "keeper"){
//                        auto rob = allRobots.front();
//                        output.insert({role.first,rob});
//                        allRobots.erase(allRobots.cbegin());
//                    }
//                    else {
//                        //auto rob = world.getRobotClosestToPoint(allRoleInfo.find(role.first)->second.getPositionToMoveTo().value(), allRobots);
//                        size_t index = 0;
//                        size_t bestIndex = 0;
//                        double closest = 9e9;
//                        for(auto robot: allRobots) {
//                            double distance = (robot->getPos().dist(allRoleInfo.find(role.first)->second.getPositionToMoveTo().value()));
//                            if (distance < closest) {
//                                closest = distance;
//                                bestIndex = index;
//                            }
//                            ++index;
//                            if(index==11) break;
//                        }
//                        output.insert({role.first, allRobots[bestIndex]});
//                        auto iter = std::find_if(allRobots.begin(), allRobots.end(), [&](const RobotView& item){
//                            if(item->getId()==allRobots[bestIndex]->getId()) return item;
//                        });
//
//                    }
//                }
//                else{
//                    auto rob = allRobots.back();
//                    output.insert({role.first,rob});
//                    allRobots.erase(allRobots.cend()-1);
//                }
//            }
//        }
//        return output;
//    }

    /**
     * In the case that there are valid forcedIDs stored in the FlagMap they are distributed first and removed to prevent overrides
     * All forcedIDs are stored in a vector and returned to track the allRobots indices that have been shifted
     * @param allRobots the vector storing the robots detected on the field
     * @param allRoles the map storing the flags and priorities of the roles set by the Play
     * @param output the map storing the roleNames and RobotViews corresponding to the new assignments
     * @return vector of forcedIDs
     */
    vector<int> Dealer::removeForcedIDs(vector<RobotView> &allRobots, FlagMap &allRoles, AssignmentMap &output) {
//        if(allRoles.contains("keeper")) {
//            output.insert({allRoles.find("keeper")->first, allRobots[allRobots.size() - 1]});
//            allRobots.erase(allRobots.end() - 1);
//            allRoles.erase("keeper");
//        }
        // track all forced robot IDs
        vector<int> forcedRobots{};
        for (const auto &role: allRoles) {
            if (role.second.forcedID != -1) {
                // if forcedID is valid add it to the output and tracking vector
                output.insert({role.first, allRobots[role.second.forcedID]});
                forcedRobots.push_back(role.second.forcedID);
                // as well as remove the robot and role to prevent unintentional override of the assignment
                allRobots.erase(allRobots.begin() + role.second.forcedID);
                allRoles.erase(role.first);
            }
        }
        return forcedRobots;
    }

    /**
     * In the case that there are not enough robots for allRoles the least important roles are removed until the number of roles and robots is the same
     * An iterator runs over the PriorityOrder backwards and erases roles of iterators priority
     * @param allRobots the vector storing the robots detected on the field
     * @param allRoles the map storing the flags and priorities of the roles set by the Play
     * @return number of missing robots in case an error occurred
     */
    int Dealer::removeRoles(vector<RobotView> &allRobots, FlagMap &allRoles) {
        RTT_INFO("Not enough robots for all roles.")
        auto missingRobots = allRoles.size() - allRobots.size();
        for (auto i = PriorityOrder.end(); i != PriorityOrder.begin(); --i) {
            for (const auto &role: allRoles) {
                if (role.second.priority == *i) {
                    allRoles.erase(role.first);
                    --missingRobots;
                    if (missingRobots == 0) return 0;
                }
            }
        }
        return static_cast<int>(missingRobots);
    }

    /**
     * Populates a cost matrix [robots (row), roles (column)] with costs based on the needed distance to the target and suitability to the role
     * Calculates the distanceCost and roleCost for every robot, for every role (stored in 2-D vector)
     * Applies a normalizer to combine the distance and role costs
     * Runs on O(n^2) time-complexity (n = number of robots that need to be assigned to a role)
     * @param allRobots the vector storing the robots detected on the field
     * @param allRoles the map storing the flags and priorities of the roles set by the Play
     * @param allRoleInfo the map storing the last stored info for the roles set by the Play (like the target position or the last assigned robot)
     * @return cost matrix for linear assignment problem
     * @warning assumes that there are the same number of robots as roles
     */
    Matrix Dealer::getCostMatrix(const vector<RobotView> &allRobots, const FlagMap &allRoles, const StpMap &allRoleInfo) {
        Matrix costs(allRobots.size(), vector<int>(allRobots.size()));

        int r{0};
        for (const auto &[roleName, flag]: allRoles) {
            int c{0};
            for (const auto &robot: allRobots) {
               // if(allRoleInfo.find(roleName) == allRoleInfo.end()) RTT_ERROR("!Unable to find necessary info for distance.")
                if (allRoleInfo.find(roleName) != allRoleInfo.end()) {
                    auto distanceCost{getCostForDistance(robot, allRoleInfo.find(roleName)->second)};
                    auto roleCost{getCostForRole(robot, flag.flags, allRoleInfo.find(roleName)->second)};
                    auto totalCost = roleCost.sumCosts + distanceCost;
                    costs[r][c] = distanceCost*100;//(distanceCost/ getWeightForPriority(flag.priority))*100;
                    cout << costs[r][c] << " ";
                }
                ++c;
            }
            ++r;
            cout << endl;
        }
        cout << endl;
        return costs;
    }

    /**
     * Retrieves the distance of a robot to a desired position
     * @param robot the RobotView with a pointer to the robot
     * @param roleInfo the last stored info for a role like the target position or the last assigned robot
     * @return cost for distance
     */
    double Dealer::getCostForDistance(const RobotView &robot, const StpInfo &roleInfo) {
        double distance{0.0};
        auto current = robot->getPos();
        auto target = current;

        if(roleInfo.getCurrentWorld() != nullptr && roleInfo.getCurrentWorld()->getHistoryWorld(5).has_value()) {
            cout << "current pos" << current << endl;
            auto wo = *roleInfo.getCurrentWorld()->getHistoryWorld(5).value();
            current = wo.getUs()[robot->getId()].get()->getPos();
            cout << "old pos" << current << endl;
        }

        if(roleInfo.getPositionToMoveTo().has_value()) {
            target = roleInfo.getPositionToMoveTo().value();
        }
        else if(roleInfo.getPositionToDefend().has_value()) {
            target = roleInfo.getPositionToDefend().value();
        }
        else if (roleInfo.getEnemyRobot().has_value()) {
            target = roleInfo.getEnemyRobot().value()->getPos();
        }
        distance += current.dist(target);
        // if current is not in the same quadrant as target add penalty to the distance
        if(current.x<0 && target.x>0){
            distance *= 2;
        }
        if(current.x>0 && target.x<0){
            distance *= 2;
        }
        if(current.y<0 && target.y>0){
            distance *= 2;
        }
        if(current.y>0 && target.y<0){
            distance *= 2;
        }
        if(FieldComputations::pointIsInDefenseArea(*m_field, target) && !FieldComputations::pointIsInDefenseArea(*m_field, current)){
            distance *= 2;
        }
//        if(roleInfo.getRobot().has_value() && roleInfo.getRobot()->get()->getPos().dist(robot->getPos()) < .5){
//            distance /= 10;
//        }


        return getCostForDistance(distance);
    }

    /**
     * Calculates the cost of a distance relative to the field
     * @param distance well... the distance
     * @return cost for distance (distance / fieldDiagonalLength)
     */
    double Dealer::getCostForDistance(double distance) {
        if(distance <= control_constants::ROBOT_RADIUS) distance = 0;
        return distance;
    }

    /**
     * Calculates the cost of all flags for a role based on a given robot
     * This cost factors into the cost matrix calculation with the distance cost
     * @param robot the RobotView of a robot
     * @param roleFlags the vector containing 'strategies' assigned to a role (max 9 total) *see Plays for flag assignment*
     * @param roleInfo the last stored info for a role like the target position or the last assigned robot
     * @return the sum of costs for all roleFlags (based on the robot) multiplied by the sum of the role's flag priorities as well as the sum all priorities for reference
     */
    Dealer::RoleCost Dealer::getCostForRole(const RobotView &robot, const vector<FlagInstruction> &roleFlags, const StpInfo &roleInfo) {
        double robotCost = 0;
        double sumWeights = 0;
        for (auto flag: roleFlags) {
            const auto costs{getDefaultFlagCosts(robot, flag, roleInfo)};
            const auto weight{getWeightForPriority(flag.level)};
            const auto weighted_costs{costs / weight};
            // sum the costs and weights
            robotCost += weighted_costs;
            sumWeights += weight;
        }
        return {robotCost, sumWeights};  // [cost,sum of weights]
    }

    /**
     * Translates a priority into a double
     * @param priority the role priority (1 of 5)
     * @return priority weight TODO:: these values need to be tuned (magic numbers)
     */
    double Dealer::getWeightForPriority(const Priority &level) {
        switch (level) {
            case Priority::KEEPER:
                return 6;
            case Priority::REQUIRED:
                return 5;
            case Priority::HIGH_PRIORITY:
                return 4;
            case Priority::MEDIUM_PRIORITY:
                return 3;
            case Priority::LOW_PRIORITY:
                return 2;
            default:
                return 1;
        }
        RTT_WARNING("Unhandled RolePriority!")
        return 0;
    }

    /**
     * Translates a boolean into binary (double)
     * @param property copy of the boolean storing the state of a robot's property
     * @return return 0 if the property is true and 1 if the property is false
     */
    double Dealer::costForProperty(bool property) {
        return property ? 0.0 : 1.0;
    }

    /**
     * Translates a role's flag/instruction into a double based on how close or fitting it is to a robot
     * @param robot the RobotView with a pointer to the robot
     * @param flag the 'strategy' assigned to a role
     * @param roleInfo the last stored info for a role like the target position or the last assigned robot
     * @return a value/cost measuring a flag's match to a robot (lower=better)
     * @note This function is virtual such that it can be mocked in the tests. The performance hit is minimal (in the scope of nanoseconds)
     */ /// TODO:: these values need to be tuned.
    double Dealer::getDefaultFlagCosts(const RobotView &robot, const FlagInstruction &flag, const StpInfo &roleInfo) {
        switch (flag.title) {
            case Flag::NOT_IMPORTANT:
                return 1;
            case Flag::KEEPER:
                // 0 if the robot is keeper and 1 if not
                return costForProperty(robot->getId() == GameStateManager::getCurrentGameState().keeperId);
            case Flag::WITH_WORKING_BALL_SENSOR:
                // 0 if the robot has a functional ball sensor and 1 if not
                return costForProperty(robot->isWorkingBallSensor());
            case Flag::WITH_WORKING_DRIBBLER:
                // 0 if the robot has a functional dribbler and 1 if not
                return costForProperty(robot->isWorkingDribbler());
            case Flag::CLOSE_TO_THEIR_GOAL:
                // distance cost to their goal
                return getCostForDistance(FieldComputations::getDistanceToGoal(*m_field, false, robot->getPos()));
            case Flag::CLOSE_TO_OUR_GOAL:
                // distance cost to our goal
                return getCostForDistance(FieldComputations::getDistanceToGoal(*m_field, true, robot->getPos()));
            case Flag::CLOSE_TO_BALL:
                // distance cost to the ball
                return getCostForDistance(robot->getDistanceToBall());
            case Flag::CLOSEST_TO_BALL:
                // 0 if robot is closest to the ball (out of all friendly robots) and 1 if not
                return costForProperty(
                        robot->getId() == m_world.getRobotClosestToBall(rtt::world::us)->get()->getId());
            case Flag::CLOSE_TO_POSITIONING:
                // distance cost to target position
                return getCostForDistance(robot, roleInfo);
            case Flag::READY_TO_INTERCEPT_GOAL_SHOT: // TODO:: this method can be improved by choosing a better line for the interception.
                // distance cost to line between ball and goal
                LineSegment lineSegment{m_world.getBall()->get()->getPos(), m_field->getOurGoalCenter()};
                return getCostForDistance(lineSegment.distanceToLine(robot->getPos()));
        }
        RTT_WARNING("Unhandled Flag Instruction! Unable to retrieve a default flag cost.")
        return 0;
    }
}