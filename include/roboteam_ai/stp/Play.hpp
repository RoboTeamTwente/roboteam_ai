//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAY_HPP
#define RTT_PLAY_HPP

#include <array>

#include "Role.hpp"
#include "stp/evaluations/BaseEvaluation.h"
#include "utilities/Dealer.h"
#include "world/World.hpp"
#include "PlayEvaluator.h"
#include "computations/PositionComputations.h"
#include "constants/GeneralizationConstants.h"

namespace rtt::ai::stp {
using pos = PositionComputations;
using eval = GlobalEvaluation;

/**
 * Play class that's used in the STP model
 * on update traverses every Role, and updates it.
 */
class Play {
   public:
    /**
     * Saves all necessary information (that is needed for a potential next Play), when this Play will be finished
     * @return Map of all the necessary information
     */
    virtual void storePlayInfo(gen::PlayInfos& previousPlayInfo) noexcept;

    /**
     * Invariant vector that contains invariants that need to be true to continue execution of this play
     */
    std::vector<GlobalEvaluation> keepPlayEvaluation;

    /**
     * Invariant vector that contains invariants that need to be true to start this play
     */
    std::vector<GlobalEvaluation> startPlayEvaluation;

    /**
     * Initializes stpInfos struct, distributes roles, sets the previousRobotNum variable and calls onInitialize()
     */
    void initialize(gen::PlayInfos& previousPlayInfo) noexcept;

    /**
     * Updates the stored world pointer and after that, updates the field instance using the updated world pointer
     * @param pointer to World
     */
    void updateWorld(world::World* world) noexcept;

    /**
     * Updates (or ticks) all the roles that have robots assigned to them
     */
    virtual void update() noexcept;

    /**
     * Calculates all the info the roles need in order to execute correctly.
     * This is a purely virtual function, so it is implemented in every play.
     */
    virtual void calculateInfoForRoles() noexcept = 0;

    /**
     * Calculate info for roles that are used in the scoring of the play.
     * This is a purely virtual function, so it is implemented in every play.
     */
    virtual void calculateInfoForScoredRoles(world::World* world) noexcept = 0;

    /**
     * Function  for in between plays calculation of score through the PlayEvaluator.
     * Using the struct PlayEvaluator::PlayScoring(uint8_t score, double weight) the factors the be considered
     * can be defined for scoring a play.
     * @param playEvaluator with the world
     * @return a final score for the play
     */
    virtual uint8_t score(PlayEvaluator& playEvaluator) noexcept = 0;

    /**
     * Virtual default dtor, ensures proper destruction of derived plays
     */
    virtual ~Play() = default;

    /**
     * Default ctor, proper construction
     */
    Play() = default;

    /**
     * Default move-ctor, ensures proper move-construction of Play
     */
    Play(Play&& other) = default;

    /**
     * Check if the preconditions of this play are true
     * @return true if the play is allowed to be started, else false
     */
    [[nodiscard]] bool isValidPlayToStart(PlayEvaluator& playEvaluator) const noexcept;

    /**
     * Check if the invariants necessary to keep this play are true
     * @return true if the play is valid to keep, else false
     */
    [[nodiscard]] virtual bool isValidPlayToKeep(PlayEvaluator& playEvaluator) noexcept;

    /**
     * Getter for the role -> status mapping
     * @return The internal role -> status mapping, roleStatuses
     */
    [[nodiscard]] std::unordered_map<Role*, Status> const& getRoleStatuses() const;

    /**
     * Gets the current play name
     */
    virtual const char* getName() = 0;

    /**
     * If score was calculated, save here
     */
    std::optional<uint8_t> lastScore;

    /**
     * Get score
     * @return score if no value -> 0
     */
    uint8_t getLastScore() const;

protected:
    /**
     * The roles, constructed in ctor of a play
     */
    std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()> roles;

    /**
     * The evaluations with their weight
     */
    std::vector<PlayEvaluator::PlayScoring> scoring;

    /**
     * Map that keeps track of the status of each role.
     * It's a Role*, because that's hashable and a unique identifier
     */
    std::unordered_map<Role*, Status> roleStatuses;

    /**
     * The stpInfos, constructed in distributeRoles
     * The string is the role_name to be able to update the info in the right role
     */
    std::unordered_map<std::string, StpInfo> stpInfos;

    /**
     * The world pointer
     */
    rtt::world::World* world{};

    /**
     * The Field
     */
    rtt::world::Field field;

    /**
     * Decides the input for the robot dealer. The result will be used to distribute the roles
     * @return a mapping between roles and robot flags, used by the robot dealer to assign roles
     */
    virtual Dealer::FlagMap decideRoleFlags() const noexcept = 0;

    /**
     * This function is used to determine if -- when a role is in an endTactic -- the endTactic should be skipped.
     * An example could be BlockRobot and Intercept. You block a robot (endTactic) until a ball is shot and then the robot
     * closest to the ball should try to intercept (skip the BlockRobot tactic to execute Intercept)
     */
    //virtual bool shouldRoleSkipEndTactic() = 0;

    /**
     * Map that holds info from the previous play
     */
    std::optional<gen::PlayInfos> previousPlayInfos;

    /**
     * Function to initialize roles and make stpInfos map
     */
    void initRoles() noexcept;

private:
    /**
     * This function refreshes the RobotViews, the BallViews and the Fields for all stpInfos.
     * This is necessary because the views are stored for a limited time; not refreshing will lead to UB
     */
    void refreshData() noexcept;

    /**
     * Assigns robots to roles
     */
    void distributeRoles() noexcept;

    /**
     * Re-calculates info for roles and reassigns robots.
     * This function is only used when the amount of robots in the field changed compared to the previous tick
     */
    void reassignRobots() noexcept;

    /**
     * The previous amount of robots
     * This is used to check if we need to re-deal (if a robot disappears for example)
     */
    int previousRobotNum{};

    /**
     * Optional function to force end plays
     * @return True if play should end this tick
     */
    bool shouldEndPlay() noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAY_HPP
