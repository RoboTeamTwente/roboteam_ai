//
// Created by john on 1/14/20.
//

#include "world/views/RobotView.hpp"

#include "world/World.hpp"

namespace rtt::world::view {
RobotView::RobotView(const rtt::world::robot::Robot *const _ptr) noexcept : robotPtr{_ptr} {}

robot::Robot const *RobotView::get() const noexcept { return robotPtr; }

robot::Robot const &RobotView::operator*() const noexcept { return *get(); }

robot::Robot const *RobotView::operator->() const noexcept { return get(); }

RobotView::operator bool() const noexcept { return get() != nullptr; }

}  // namespace rtt::world::view
