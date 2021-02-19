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
    KEEPER
};

// Lowest to Highest Priority ordering. UNIQUE Classes are the Highest. (Order using in Dealer::distribute() )
enum class DealerFlagPriority { LOW_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, REQUIRED, KEEPER };

class Dealer {
    FRIEND_TEST(DealerTest, it_properly_distributes_robots);
    FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

   public:
    struct DealerFlag {
        DealerFlagTitle title;
        DealerFlagPriority priority;
        explicit DealerFlag(DealerFlagTitle title, DealerFlagPriority priority);
    };
    using FlagMap = std::map<std::string, std::pair<DealerFlagPriority, std::vector<DealerFlag>>>;
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

    /**
     * Calculates the score for a flag by multiplying the factor and score
     * The factor is based on the priority and the score is based on the trueness of a property
     * @param robot
     * @param flag
     * @return Score for a flag
     */
    std::pair <double, double> getScoreForFlag(v::RobotView robot, DealerFlag flag);

    /**
     * Calculates the score for a distance between a point and a robot
     * @param stpInfo
     * @param robot
     * @return Score for distance
     */
    double getScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot);

    /**
     * Populates the matrix of robots and roles with scores based on flags and distance
     * @param allRobots
     * @param flagMap
     * @param stpInfoMap
     * @return The score matrix
     */
    std::vector<std::pair<std::vector<double>, int>> getScoreMatrix(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                                                    const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

    /**
     * Translates a priority into a double
     * @param flag
     * @return priority factor
     */
    static double getFactorForPriority(const DealerFlagPriority &flagPriority);

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
     * @return the score of all flags combined
     */
    std::pair <double, double> scoreForFlags(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot);

    /**
     * Makes a mapping from roleName to a robot using the result of the hungarian
     * @param allRobots
     * @param flagMap
     * @param assignment
     * @return Returns the mapping
     */
    std::unordered_map<std::string, v::RobotView> mapFromAssignments(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap, const std::vector<int> &assignment) const;
};
}  // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
