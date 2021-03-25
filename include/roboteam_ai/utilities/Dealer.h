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

// Set up a struct for dealerflags. Set up a struct for dealerflags.
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

enum class DealerFlagPriority { LOW_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, REQUIRED, KEEPER };
static std::vector<DealerFlagPriority> PriorityOrder { DealerFlagPriority::KEEPER,
                                                       DealerFlagPriority::REQUIRED,
                                                       DealerFlagPriority::HIGH_PRIORITY,
                                                       DealerFlagPriority::MEDIUM_PRIORITY,
                                                       DealerFlagPriority::LOW_PRIORITY };

class Dealer {
    FRIEND_TEST(DealerTest, it_properly_distributes_robots);
    FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

   public:
    struct DealerFlag {
        DealerFlagTitle title;
        DealerFlagPriority priority;
        explicit DealerFlag(DealerFlagTitle title, DealerFlagPriority priority);
    };

    struct RoleInfo {
        DealerFlagPriority priority;
        std::vector<DealerFlag> flags;
        int forcedID = -1;
    };

    using FlagMap = std::map<std::string, RoleInfo>;
    Dealer(v::WorldDataView world, rtt::world::Field *field);
    virtual ~Dealer() = default;  // needed for test
    // Create a distribution of robots
    std::unordered_map<std::string, v::RobotView> distribute(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                                                             const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

   protected:
    // This function is virtual such that it can be mocked in the tests.
    // the performance hit is minimal (in the scope of nanoseconds)
    virtual double getDefaultFlagScores(const v::RobotView &robot, const DealerFlag &flag);

   private:
    v::WorldDataView world;
    rtt::world::Field *field;

    struct FlagScore {
        double score;
        double weight;
    };

    struct RobotRoleScore {
        double sumScore;
        double sumWeights;
    };

    struct RoleScores {
        std::vector<double> robotScores;
        DealerFlagPriority priority;
    };

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

    void distribute_forcedIDs(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                              std::unordered_map<std::string, v::RobotView> &output);
};
}  // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
