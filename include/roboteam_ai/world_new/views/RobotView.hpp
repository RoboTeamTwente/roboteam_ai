//
// Created by john on 1/14/20.
//

#ifndef RTT_ROBOT_VIEW_HPP
#define RTT_ROBOT_VIEW_HPP

#include <include/roboteam_ai/utilities/Constants.h>
#include "world_new/Robot.hpp"

namespace rtt::world_new::view {

    /**
     * Class that offers a read-only memory view of the Robots
     */
    class RobotView {
        robot::Robot const* const rbt;
    public:
        /**
         * Constructs a RobotView from a Robot
         * @param _ptr Robot pointer to construct from
         * @ensures After contruction this->rbt == _ptr;
         */
        explicit RobotView(robot::Robot const* _ptr) noexcept;

        /**
         * Gets the internal pointer to the robot (std::unique_ptr style)
         * @return Returns the internal pointer
         */
        [[nodiscard]] robot::Robot const* get() const noexcept;

        /**
         * Overloads the dereference operator (*ptr) in std::unique_ptr style
         * @return Returns a constant reference to the non-owned Robot
         */
        robot::Robot const& operator*() const noexcept;

        /**
         * Gives std::unique_ptr (std::unique_ptr<Robot>()->genevaDriver) style access into member funcntions
         * @return Returns get()
         */
        robot::Robot const* operator->() const noexcept;

        /**
         * Copy assignment operator
         * @return *this
         */
        RobotView& operator=(RobotView const&) noexcept;

        /**
         * Move assignment operator
         * @return *this
         */
        RobotView& operator=(RobotView&&) noexcept;

        /**
         * Copy constructor
         */
        RobotView(RobotView const&) = default;

        /**
         * Move constructor, same as copy ctor
         */
        RobotView(RobotView&&) noexcept;

        /**
         * Check whether the current robot has the ball
         * @param maxDist maximum distance for ball posession
         * @return true if dist(ball, robot) < maxDist else false
         */
        [[nodiscard]] bool hasBall(double maxDist = ai::Constants::MAX_BALL_BOUNCE_RANGE()) const noexcept;

        /**
         * Gets the kicker for the Robot that this view is viewing
         * @return A vector2 representation of the kicker
         */
        [[nodiscard]] Vector2 getKicker() const noexcept;
    };

}

#endif //RTT_ROBOT_VIEW_HPP
