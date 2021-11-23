/**
 * The Dealer fulfills the task of distributing roles set in the Play to the available robots on the field
 * In order to have an optimized distribution of robots to roles this class uses the aid of a Linear Assignment Algorithm
 * Hungarian.h referred to as the Munkres Assignment or otherwise know as the Hungarian Algorithm
 * This class works very closely with Play.cpp as well as its inheritors
 * @note refurbished by Max Thielen on 03/11/21
 */

#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_

#include <iostream>
#include <map>
#include <vector>
#include <iterator>
#include "stp/StpInfo.h"
#include "world/World.hpp"
#include "world/Field.h"
#include "world/views/RobotView.hpp"
#include "world/views/WorldDataView.hpp"
#include "world/FieldComputations.h"
#include "roboteam_utils/Hungarian.h"
#include "roboteam_utils/Print.h"
#include "utilities/GameStateManager.hpp"
#include "gtest/gtest_prod.h"

namespace rtt::ai::DealerSpecific {
    /// Flags are like strategies for Roles and are declared in the Play inheritors
    enum class Flag {
        KEEPER,
        CLOSE_TO_BALL,
        CLOSEST_TO_BALL,
        CLOSE_TO_POSITIONING,
        READY_TO_INTERCEPT_GOAL_SHOT,
        CLOSE_TO_THEIR_GOAL,
        CLOSE_TO_OUR_GOAL,
        WITH_WORKING_BALL_SENSOR,
        WITH_WORKING_DRIBBLER,
        NOT_IMPORTANT
    };
    /// The priority hierarchy is used to influence the cost of assigning a role to a robot
    /// as well as ensure the most important roles are not neglected over more trivial ones
    enum class Priority { LOW_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, REQUIRED, KEEPER };
    static vector<Priority> PriorityOrder {
            Priority::KEEPER,
            Priority::REQUIRED,
            Priority::HIGH_PRIORITY,
            Priority::MEDIUM_PRIORITY,
            Priority::LOW_PRIORITY
    };
}  // namespace rtt::ai::DealerSpecific

namespace rtt::ai{
    using namespace std;
    using namespace stp;
    using namespace rtt::world;
    using namespace rtt::world::view;
    using namespace DealerSpecific;

    class Dealer{
    public:
        Dealer(WorldDataView world, Field *field);
        virtual ~Dealer();  // Testing Reasons

        /// Stores a role's strategy in combination with the strategy's priority
        struct FlagInstruction {
            explicit FlagInstruction(Flag title, Priority level);
            Flag title;
            Priority level;
        };
        /// Stores all 'strategies' in a vector and enables forcing a robot with a given ID to take its role
        struct RoleInstruction {
            Priority priority;
            vector<FlagInstruction> flags;
            int forcedID = -1;  //-1 when this should not be considered
        };
        /// Just makes everything a lot easier to read :]
        using FlagMap = map<string, RoleInstruction>;
        using StpMap = unordered_map<string, StpInfo>;
        using AssignmentMap = unordered_map<string, RobotView>;
        using Matrix = vector<vector<int>>;

        AssignmentMap distribute(vector<RobotView> allRobots, FlagMap allRoles, const StpMap &allRoleInfo);

    private:
        /// access to vision data for calculating distances and such
        WorldDataView  m_world;
        rtt::world::Field *m_field;
        /// used as a return type to pass both values at the same time
        struct RoleCost {
            double sumCosts;
            double sumWeights;
        };

        vector<int> removeForcedIDs(vector<RobotView> &allRobots, FlagMap &allRoles, AssignmentMap &output);
        int removeRoles(vector<RobotView>& allRobots, FlagMap& allRoles);
        Matrix getCostMatrix(const vector<RobotView> &allRobots, const FlagMap &allRoles, const StpMap &allRoleInfo);
        double getCostForDistance(const RobotView &robot, const StpInfo &roleInfo);
        double getCostForDistance(double distance);
        RoleCost getCostForRole(const RobotView &robot, const vector<FlagInstruction> &roleFlags, const StpInfo &roleInfo);
        static double getWeightForPriority(const Priority &level);
        static double costForProperty(bool property);

        static double Vdistance(rtt::Vector2 current, rtt::Vector2 target){
            //cout << "current: " << current.x << " " << current.y << " target: " << target.x << " " << target.y << endl;
            auto distX = target.x-current.x;
            auto distY = target.y-current.y;
            return sqrt(distX*distX + distY*distY);
        }

        FRIEND_TEST(DealerTest, it_properly_distributes_robots);
        FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

    protected:
        virtual double getDefaultFlagCosts(const RobotView &robot, const FlagInstruction &flag, const StpInfo &roleInfo);
    };
} // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
