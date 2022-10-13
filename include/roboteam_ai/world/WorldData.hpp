//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_DATA_HPP
#define RTT_WORLD_DATA_HPP

#include <proto/RobotFeedback.pb.h>
#include <proto/Setting.pb.h>
#include <proto/World.pb.h>

#include <optional>
#include <vector>

#include "utilities/Settings.h"
#include "world/views/RobotView.hpp"

namespace rtt::world {
class World;

namespace robot {
class Robot;
}  // namespace robot

/**
 * WorldData structs
 * Hold data about a specific time point in the world.
 * Holds:
 *  Robots
 *  Ball
 *  Timepoint
 */
class WorldData {
    friend class World;

   private:
    /**
     * Constructs new world data
     * @param protoMsg Proto message to construct he world from
     * @param settings Settings for team stuff etc
     * @param feedback Feedback to apply to robots that'll be constructed
     *
     * Ownership is taken of protoMsg
     */
    WorldData(const World *data, proto::World &protoMsg, rtt::Settings const &settings) noexcept;

    /**
     * Owning container of robots
     */
    std::vector<rtt::world::robot::Robot> robots;

    /**
     * Non owning vector of views
     */
    std::vector<world::view::RobotView> robotsNonOwning = {};

    /**
     * Non-owning container of Robot const* const's (aka RobotView) for our team
     */
    std::vector<world::view::RobotView> us = {};

    /**
     * Non-owning container of RobotViews of the enemy team
     */
    std::vector<world::view::RobotView> them = {};

    /**
     * Optional ball, None variant if not visible
     */
    std::optional<rtt::world::ball::Ball> ball;

    /**
     * Timestamp identical to the protobuf message's time()
     */
    uint64_t time{};

   public:
    /**
     * Default ctor for default STL container initialization
     */
    WorldData() = default;

    /**
     * Copy assignment operator and constructor
     * Copy over members and set the view vectors
     */
    WorldData &operator=(WorldData const &) noexcept;
    WorldData(WorldData const &) noexcept;

    /**
     * Move constructor, simply moves all members
     * @param old Data to move
     */
    WorldData(WorldData &&old) = default;

    /**
     * Move assignment operator
     * @return *this
     */
    WorldData &operator=(WorldData &&) = default;

    /**
     * Does exactly what it says, it sets the internal vectors for the views
     */
    void setViewVectors() noexcept;

    /**
     * Gets a non-owning container of robots that are in our team
     * @return this->us
     */
    [[nodiscard]] std::vector<view::RobotView> const &getUs() const noexcept;

    /**
     * Gets a non-owning container of robots that are in the enemy team
     * @return this->them
     */
    [[nodiscard]] std::vector<view::RobotView> const &getThem() const noexcept;

    /**
     * Gets a Some or None non-owning variant of a Ball, aka a BallView
     * @return ball ? Some(BallView) : None
     */
    [[nodiscard]] std::optional<view::BallView> getBall() const noexcept;

    /**
     * Gets the internal vector of robotviews
     * @return A const& to it
     */
    [[nodiscard]] const std::vector<view::RobotView> &getRobotsNonOwning() const noexcept;

    /**
     * Gets the time for the current contained world
     * @return
     */
    [[nodiscard]] uint64_t getTime() const noexcept;
};
}  // namespace rtt::world

#endif  // RTT_WORLD_DATA_HPP
