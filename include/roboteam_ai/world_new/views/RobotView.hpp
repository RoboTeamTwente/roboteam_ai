//
// Created by john on 1/14/20.
//

#ifndef RTT_ROBOT_VIEW_HPP
#define RTT_ROBOT_VIEW_HPP

#include <include/roboteam_ai/utilities/Constants.h>
#include "world_new/Robot.hpp"
#include "world_new/RobotControllers.hpp"

namespace rtt::world_new::view {

/**
 * Class that offers a read-only memory view of the Robots
 *
 * Imagine RobotView being to Robot what string_view is to std::string,
 * except it provides more utility functions
 */
class RobotView {
    robot::Robot const *robotPtr;

   public:
    /**
     * Constructs a RobotView from a Robot
     * @param _ptr Robot pointer to construct from
     * @ensures After contruction this->rbt == _ptr;
     */
    explicit RobotView(robot::Robot const *_ptr) noexcept;

    /**
     * @return get() != null;
     */
    explicit operator bool() const noexcept;

    /**
     * Gets the internal pointer to the robot (std::unique_ptr style)
     * @return Returns the internal pointer
     */
    [[nodiscard]] robot::Robot const *get() const noexcept;

    /**
     * Overloads the dereference operator (*ptr) in std::unique_ptr style
     * @return Returns a constant reference to the non-owned Robot
     */
    robot::Robot const &operator*() const noexcept;

    /**
     * Gives std::unique_ptr (std::unique_ptr<Robot>()->genevaDriver) style access into member funcntions
     * @return Returns get()
     */
    robot::Robot const *operator->() const noexcept;

    /**
     * Copy assignment operator
     * @return *this
     */
    RobotView &operator=(RobotView const &) = default;

    /**
     * Move assignment operator
     * @return *this
     */
    RobotView &operator=(RobotView &&) = default;

    /**
     * Copy constructor
     */
    RobotView(RobotView const &) = default;

    /**
     * Move constructor, same as copy ctor
     */
    RobotView(RobotView &&) = default;

    ~RobotView() = default;

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

    /**
     * Gets the controllers for a robot
     * @return World->getControllersForRobot(robotPtr->id);
     */
    [[nodiscard]] robot::RobotControllers &getControllers() const noexcept;
};

}  // namespace rtt::world_new::view

#endif  // RTT_ROBOT_VIEW_HPP
