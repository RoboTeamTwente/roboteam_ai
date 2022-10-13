//
// Created by john on 1/6/20.
//

#ifndef RTT_WORLD_DATA_VIEW_HPP
#define RTT_WORLD_DATA_VIEW_HPP

#include <vector>

#include "BallView.hpp"
#include "RobotView.hpp"
#include "utilities/Constants.h"
#include "world/Ball.hpp"
#include "world/Robot.hpp"
#include "world/Team.hpp"

namespace rtt::world {
class WorldData;
}

namespace rtt::world::view {
/**
 * A non-owning view of WorldData, with the utility functions that roboteam uses
 * Taking ownership of anything that this provides will result in
 * undefined behavior.
 */
class WorldDataView {
   public:
    /**
     * @return get() != nullptr
     */
    explicit operator bool() const noexcept;

    /**
     * Constructs a WorldDataView from a WorldData const*
     * @param _ptr Pointer to construct from
     *
     * Pointer is asserted in debug
     */
    explicit WorldDataView(WorldData const *_ptr) noexcept;

    /**
     * Overloads the dereference operator
     * @return Returns a constant reference to the contained data
     */
    [[nodiscard]] WorldData const &operator*() const noexcept;

    /**
     * Overloads the dereference (->) operator to allow for std::unique_ptr style member access
     * @return get();
     */
    [[nodiscard]] WorldData const *operator->() const noexcept;

    /**
     * Gets the internal data pointer
     * @return this->data
     */
    [[nodiscard]] WorldData const *get() const noexcept;

    /**
     * Gets our own robots
     * @return data->getUs();
     */
    [[nodiscard]] std::vector<view::RobotView> const &getUs() const noexcept;

    /**
     * Gets the enemies their robots
     * @return data->getThem();
     */
    [[nodiscard]] std::vector<view::RobotView> const &getThem() const noexcept;

    /**
     * Gets the optional ball
     * @return data->getBall();
     */
    [[nodiscard]] std::optional<view::BallView> getBall() const noexcept;

    /**
     * Gets a robot for an id
     * @param id Robot's ID of the robot to get
     * @param ourTeam true if the robot should be fetched from our team, false if from enemies
     * @return data->getRobotForId(id, ourTeam);
     */
    [[nodiscard]] std::optional<view::RobotView> getRobotForId(uint8_t id, bool ourTeam = true) const noexcept;

    /**
     * Copy assignment operator, does nothing
     */
    WorldDataView &operator=(WorldDataView const &o) = default;

    /**
     * Move assignment operator, does nothing
     */
    WorldDataView &operator=(WorldDataView &&o) = default;

    /**
     * Copy constructor
     * @param o Object to copy
     */
    WorldDataView(WorldDataView const &o) = default;

    /**
     * Move constructor, same as copy ctor
     * @param o Object to move
     */
    WorldDataView(WorldDataView &&o) = default;

    /**
     * Gets a non-owning container of Robot*, aka RobotView
     * Complexity: O(n) -> do not use too often.
     * @return A vector of robots
     */
    [[nodiscard]] const std::vector<RobotView> &getRobotsNonOwning() const noexcept;

    /**
     * Gets a view of the closest robot to a point
     * @param point Point to check to
     * @param team `us` if it should be fetched from our team, `them` if theirs, `both` if both teams
     * @return An std::optional of a non-owning view of the robot or an std::nullopt
     */
    [[nodiscard]] std::optional<RobotView> getRobotClosestToPoint(const Vector2 &point, Team team = both) const noexcept;

    /**
     * Gets a view of the robot closes to a point
     * @param point Point to check to
     * @param robots Vector of RobotViews
     * @return An std::optional of a non-owning view of the robot or an std::nullopt
     */
    [[nodiscard]] std::optional<RobotView> getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotView> &robots) const noexcept;

    /**
     * Gets the robot closest to a ball
     * @param team Team to fetch from
     * @return An std::optional of a non-owning view of the robot or an std::nullopt
     */
    [[nodiscard]] std::optional<RobotView> getRobotClosestToBall(Team team = both) const noexcept;

    /**
     * Gets a view over the robot that currently has the ball
     * @param team Team enum of team to fetch from
     * @return A non-owning view of the robot that has the ball
     */
    [[nodiscard]] std::optional<RobotView> whichRobotHasBall(Team team = both) const;

   private:
    /**
     * Constant world data that's used in the view
     */
    WorldData const *data;
};
}  // namespace rtt::world::view

#endif  // RTT_WORLD_DATA_VIEW_HPP
