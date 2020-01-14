//
// Created by john on 1/14/20.
//

#ifndef RTT_ROBOT_VIEW_HPP
#define RTT_ROBOT_VIEW_HPP

#include "world/robot.hpp"

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
    };

}

#endif //RTT_ROBOT_VIEW_HPP
