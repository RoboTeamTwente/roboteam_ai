//
// Created by john on 1/14/20.
//

#include "include/roboteam_ai/world/views/robot_view.hpp"

namespace rtt::world_new::view {
    RobotView::RobotView(const rtt::world_new::robot::Robot *const _ptr) noexcept
            : rbt{_ptr} {}

    robot::Robot const *RobotView::get() const noexcept {
        return rbt;
    }

    robot::Robot const &RobotView::operator*() const noexcept {
        return *get();
    }

    robot::Robot const *RobotView::operator->() const noexcept {
        return get();
    }
}
