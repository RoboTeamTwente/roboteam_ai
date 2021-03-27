#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_

/*
 * The purpose of Dealer is to go from pairs of (name -> dealerFlags) to pairs of (name -> robot id).
 * In other words, there is a requirement for each role in the tree, and Dealer maps the robot that fits those
 * requirements best to that role in the tree.
 * The lower the score, the better.
 */

#include <stp/StpInfo.h>

#include <iostream>
#include <map>
#include <vector>

#include "gtest/gtest_prod.h"
#include "world/Field.h"
#include "world/views/RobotView.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {
namespace v = rtt::world::view;

/// Set up a struct for dealerFlags. Set up a struct for dealerFlags.
enum class DealerFlagTitle {
    CLOSE_TO_THEIR_GOAL,
    CLOSE_TO_OUR_GOAL,
    CLOSE_TO_BALL,
    CLOSE_TO_POSITIONING,
    WITH_WORKING_BALL_SENSOR,
    WITH_WORKING_DRIBBLER,
    NOT_IMPORTANT,
    READY_TO_INTERCEPT_GOAL_SHOT,
    KEEPER,
    CLOSEST_TO_BALL
};

/// Generalization for Priority of a role
enum class DealerFlagPriority { LOW_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, REQUIRED, KEEPER };

/// The order at which the Priority will be dealt (Keeper first, Low last).
static std::vector<DealerFlagPriority> PriorityOrder { DealerFlagPriority::KEEPER,
                                                       DealerFlagPriority::REQUIRED,
                                                       DealerFlagPriority::HIGH_PRIORITY,
                                                       DealerFlagPriority::MEDIUM_PRIORITY,
                                                       DealerFlagPriority::LOW_PRIORITY };

class Dealer {
    FRIEND_TEST(DealerTest, it_properly_distributes_robots);
    FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

   public:
    /// Definition of a factor (title) that should influence the distribution with its impact (priority)
    struct DealerFlag {
        DealerFlagTitle title;
        DealerFlagPriority priority;
        explicit DealerFlag(DealerFlagTitle title, DealerFlagPriority priority);
    };

    /// The priority of the role with the falgs that need to be considered.
    // Forced ID should be ONLY be used in situations where it would bypass the inefficiency of the world to ai communication
        // i.e. ball position and velocity when passing another robot in the previous play.
    struct RoleInfo {
        DealerFlagPriority priority;
        std::vector<DealerFlag> flags;
        int forcedID = -1;  //-1 when this should not be considered
    };

    /// Map with the roleName and the RoleInfo (priority, flags and forcedID)
    using FlagMap = std::map<std::string, RoleInfo>;
    Dealer(v::WorldDataView world, rtt::world::Field *field);
    virtual ~Dealer() = default;  // needed for test

    /**
     * Distributes the role between the friendly robots considering all information given in the flagMap
     * !!! allRobots and flagMap are copies as they are modified to reduce computations after the forcedID's.
     * @param allRobots vector containing all friendly robots
     * @param flagMap information map with all information for each role (with key -> roleName)
     * @param stpInfoMap
     * @return a vector with the roleName and the Robot that should get the role
     */
    std::unordered_map<std::string, v::RobotView> distribute(std::vector<v::RobotView> allRobots, FlagMap flagMap,
                                                             const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

   protected:
    // This function is virtual such that it can be mocked in the tests.
    // the performance hit is minimal (in the scope of nanoseconds)
    virtual double getDefaultFlagScores(const v::RobotView &robot, const DealerFlag &flag);

   private:
    v::WorldDataView world;
    rtt::world::Field *field;

    /// Score of a certain flag (/factor) with its weight that it was multiplied with (to be used in normalization)
    struct FlagScore {
        double score;
        double weight;
    };

    /// Final score a specific robot with its sum of score and sum of weights (to be used in normalization)
    struct RobotRoleScore {
        double sumScore;
        double sumWeights;
    };

    /// The individual robot scores for every robot for a specific Role and the Roles priority
    struct RoleScores {
        std::vector<double> robotScores;
        DealerFlagPriority priority;
    };

    /// Used in Dealer that is re-used for each priority to save the information for that loop
    struct DealerDistribute {
        std::vector<std::vector<double>> currentScores; // Scores to be distributed (highest Priority)
        std::vector<int> newAssignments;                // Assignments from these scores
        std::vector<int> currentRoles;                  // Index of roles inside Score (role column)
        std::vector<int> originalRolesIndex;            // Index of roles from original index (role number)
        std::vector<int> currentIDs;                    // Index of ID's inside Score (robot row)
        std::vector<int> originalIDsIndex;              // Index of ID's from original index (robot id)
    };

    /**
     * Calculates the score for a flag by multiplying the factor and score
     * The factor is based on the priority and the score is based on the trueness of a property
     * @param robot
     * @param flag
     * @return Score for a flag. First is score, second is weight (for normalization)
     */
    FlagScore getRobotScoreForFlag(v::RobotView robot, DealerFlag flag);
    /**
     * Calculates the score for a distance between a point and a robot
     * @param stpInfo
     * @param robot
     * @return Score for distance
     */
    double getRobotScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot);

    /**
     * Populates the matrix of robots and roles with scores based on flags and distance
     * @param allRobots
     * @param flagMap
     * @param stpInfoMap
     * @return The score matrix
     */
    std::vector<RoleScores> getScoreMatrix(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                                                    const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

    /**
     * Translates a priority into a double
     * @param flag
     * @return priority weight
     */
    static double getWeightForPriority(const DealerFlagPriority &flagPriority);

    /**
     * Calculates the cost of travelling a certain distance
     * @param distance  the distance to travel
     * @param fieldWidth
     * @param fieldHeight
     * @return cost of travelling that distance
     */
    static double costForDistance(double distance, double fieldWidth, double fieldHeight);

    /**
     * Calculates the cost of a property (think of isWorkingDribbler())
     * @param property
     * @return cost of a property
     */
    static double costForProperty(bool property);

    /**
     * Calculates the score for all flags for a role, for one robot (so will be called 11 times for each role)
     * @param dealerFlags
     * @param robot
     * @return the score of all flags combined and the weight for how much they should be taken into account
     */
    RobotRoleScore getRobotScoreForRole(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot);

    /**
     * Initializes the reference of the variables to shorten function size.
     * Reserves the vectors and fills them with the required info.
     * @param indexRoles vector with Role numbering
     * @param indexID vector with ID numbering
     * @param roleNames vector with roleNames (order matters)
     * @param flagMap has info for the other parameters
     */
    void distribute_init(std::vector<int> &indexRoles, std::vector<int> &indexID, std::vector<std::string> &roleNames,
                         const FlagMap &flagMap);

    /**
     * Removes the occurrence of a role and robot from all fields that involve them.
     * @param current struct with the current loop link to a unique priority
     * @param indexRoles vector with Role numbering
     * @param indexID vector with ID numbering
     * @param scores vector with all Scores for Robots for each role and its priority
     */
    void distribute_remove(DealerDistribute &current, std::vector<int> &indexRoles, std::vector<int> &indexID,
                           std::vector<RoleScores> &scores);

    /**
     * Distributes the forced roles first, so that other rest of the function does not need to compute extra information
     * @param allRobots a copy of all our robots, where in the robots will be removed
     * @param flagMap wherein the roles will be removed
     * @param output
     */
    void distribute_forcedIDs(std::vector<v::RobotView> &allRobots, FlagMap &flagMap,
                              std::unordered_map<std::string, v::RobotView> &output);
};
}  // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
