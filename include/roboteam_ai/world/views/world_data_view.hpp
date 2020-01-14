//
// Created by john on 1/6/20.
//

#ifndef RTT_WORLD_DATA_VIEW_HPP
#define RTT_WORLD_DATA_VIEW_HPP

#include <vector>
#include "world/robot.hpp"
#include "world/ball.hpp"
#include "robot_view.hpp"
#include "ball_view.hpp"

namespace rtt::world_new {
    class WorldData;
}

namespace rtt::world_new::view {
    /**
     * A non-owning view of WorldData, with the utility functions that roboteam uses
     * Taking ownership of anything that this provides will result in
     * undefined behavior.
     */
    class WorldDataView {
    public:
        /**
         * Constructs a WorldDataView from a WorldData const*
         * @param _ptr Pointer to construct from
         *
         * Pointer is asserted in debug
         */
        explicit WorldDataView(WorldData const *_ptr) noexcept;

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
         * Gets all the robots in the owning container
         * @return data->getRobots();
         */
        [[nodiscard]] std::vector<rtt::world_new::robot::Robot> const &getRobots() const noexcept;

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
        [[nodiscard]] std::optional<view::RobotView>
        getRobotForId(uint8_t id, bool ourTeam = true) const noexcept;

        /**
         * Gets a non-owning container of robots with specific ids
         * @param robots A set of ID's to get the matching robots for
         * @param ourTeam true if it should be fetched from our team, false if not
         * @return A non-owning container of robots
         */
        [[nodiscard]] std::vector<view::RobotView>
        getRobotsForIds(std::set<uint8_t> const &robots, bool ourTeam = true) const noexcept;

        /**
         * Copy assignment operator, does nothing
         */
        WorldDataView& operator=(WorldDataView const& o) noexcept;

        /**
         * Move assignment operator, does nothing
         */
        WorldDataView& operator=(WorldDataView&& o) noexcept;

        /**
         * Copy constructor
         * @param o Object to copy
         */
        WorldDataView(WorldDataView const& o) = default;

        /**
         * Move constructor, same as copy ctor
         * @param o Object to move
         */
        WorldDataView(WorldDataView&& o) noexcept;

    private:
        /**
         * Constant world data that's used in the view
         */
        WorldData const *const data;
    };
}


#endif //RTT_WORLD_DATA_VIEW_HPP
