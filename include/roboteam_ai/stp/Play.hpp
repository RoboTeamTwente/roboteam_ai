//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAY_HPP
#define RTT_PLAY_HPP

#include <utilities/Constants.h>
#include <utilities/Dealer.h>

#include <array>
#include <stp/invariants/BaseInvariant.h>

#include "Role.hpp"
#include "world_new/World.hpp"

namespace rtt::ai::stp {

/**
 * Play class that's used in the STP model
 * on update traverses every Role, and updates it.
 */
class Play {
   public:
    /**
    * Invariant vector that contains invariants that need to be true to continue execution of this play
    */
    std::vector<std::unique_ptr<invariant::BaseInvariant>> keepPlayInvariants;

    /**
    * Invariant vector that contains invariants that need to be true to start this play
    */
    std::vector<std::unique_ptr<invariant::BaseInvariant>> startPlayInvariants;
	
    /**
     * Initializes tacticInfos vector and calls distributeRoles
     */
    void initialize() noexcept;

    /**
     * Updated the stored world
     * @param world
     */
    void updateWorld(world_new::World* world) noexcept;

    /**
     * Updates all the roles
     */
    virtual void update() noexcept;

    /**
     * Calculates all the info (mostly positions) the roles in this play need to execute
     */
    virtual void calculateInfoForRoles() noexcept = 0;

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlayToStart() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world_new::World::instance())
     * @return The score, 0 - 100
     */
    [[nodiscard]] virtual uint8_t score(world_new::World* world) noexcept = 0;

    /**
     * Virtual default dtor, ensures proper destruction of Play
     */
    virtual ~Play() = default;

    /**
     * Ctor that constructs a play and assigns its name
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
    [[nodiscard]] bool isValidPlayToStart(world_new::World* world) const noexcept;

    /**
     * Check if the invariants for the play to keep running are true
     * @return
     */
    [[nodiscard]] bool isValidPlayToKeep(world_new::World* world) const noexcept;

    /**
     * @return true if all roles are finished
     */
    [[nodiscard]] bool arePlayRolesFinished();

    /**
     * @return The internal role -> status mapping
     */
    [[nodiscard]] std::unordered_map<Role*, Status> const& getRoleStatuses() const;

    /**
     * Gets the current play name
     */
    virtual const char* getName() = 0;

protected:
    /**
     * The roles, constructed in ctor of a play
     */
    std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()> roles;

    /**
     * Map that keeps track of the status of each role.
     * It's a Role*, because that's hashable and only 1
     * instance exists of each role
     */
    std::unordered_map<Role*, Status> roleStatuses;

    /**
     * The stpInfos, constructed in distributeRoles
     * The string is the role_name to be able to update the info in the right role
     */
    std::unordered_map<std::string, StpInfo> stpInfos;

    /**
     * The world
     */
    rtt::world_new::World* world;

    /**
     * The Field
     */
    rtt::ai::world::Field field;

    /**
     * Decides the input to the robot dealer. The result will be used to distribute the roles
     * @return a mapping between roles and robot flags, used by the robot dealer to assign roles
     */
    virtual Dealer::FlagMap decideRoleFlags() const noexcept = 0;

    /**
     * This function is used to determine if, when a role is in an endtactic, that endtactic should be skipped.
     * An example could be BlockRobot and Intercept. You block a robot until a ball is shot and then the robot
     * closest to the ball should try to intercept
     */
    virtual bool shouldRoleSkipEndTactic() = 0;

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

    int previousRobotNum{};
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAY_HPP
