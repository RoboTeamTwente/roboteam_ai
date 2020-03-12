//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAY_HPP
#define RTT_PLAY_HPP

#include <utilities/Dealer.h>

#include <array>

#include "Role.hpp"
#include "world_new/World.hpp"

namespace rtt::ai::stp {

/**
 * Play class that's used in the STP model
 * on update traverses every Role, and updates it.
 */
class Play {
   public:
    constexpr static size_t ROBOT_COUNT = 11;

    /**
     * Initializes tacticInfos vector and calls assignRoles
     */
    void initialize() noexcept;

    /**
     * Updated the stored world
     * @param world
     */
    void updateWorld(world_new::World* world) noexcept;

    /**
     * Updates all the roles
     * @param info Information to pass down to the Roles
     * @return Status depending on return value,
     * if all finished -> finished
     * if any failed -> failed
     * if any waiting -> waiting
     * otherwise -> running
     */
    [[nodiscard]] virtual Status update() noexcept;

    /**
     * Checks whether the current play is a valid play
     * @param world World to check for (world_new::World::instance())
     * @return true if valid, false if not
     */
    [[nodiscard]] virtual bool isValidPlay(world_new::World* world) noexcept = 0;

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
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
     * Default ctor, ensures proper construction of Play
     */
    Play() = default;

    /**
     * Default move-ctor, ensures proper move-construction of Play
     */
    Play(Play&& other) = default;

   protected:
    /**
     * The roles, constructed in ctor of a play
     */
    std::array<std::unique_ptr<Role>, ROBOT_COUNT> roles;

    /**
     * The stpInfos, constructed in assignRoles
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
    rtt::ai::Field field;

    protected:
    /**
     * Assigns robots to roles
     */
    virtual void assignRoles() noexcept = 0;
};

}  // namespace rtt::ai::stp

#endif  // RTT_PLAY_HPP
