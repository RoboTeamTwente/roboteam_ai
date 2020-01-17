//
// Created by john on 1/14/20.
//

#include "world_new/views/RobotView.hpp"

#include <include/roboteam_ai/utilities/Constants.h>

namespace rtt::world_new::view {
RobotView::RobotView(const rtt::world_new::robot::Robot *const _ptr) noexcept : robotPtr{_ptr} {}

robot::Robot const *RobotView::get() const noexcept { return robotPtr; }

robot::Robot const &RobotView::operator*() const noexcept { return *get(); }

robot::Robot const *RobotView::operator->() const noexcept { return get(); }

RobotView &RobotView::operator=(RobotView const &o) noexcept {
    if (&o == this) {
        return *this;
    }
    return *this;
}

RobotView &RobotView::operator=(RobotView &&o) noexcept {
    if (&o == this) {
        return *this;
    }
    return *this;
}

RobotView::RobotView(RobotView &&o) noexcept : robotPtr{o.robotPtr} {}

bool RobotView::hasBall(double maxDist) const noexcept { return get()->isIHaveBall() && get()->getDistanceToBall() < maxDist; }

Vector2 RobotView::getKicker() const noexcept {
    Vector2 distanceToKicker{ai::Constants::CENTRE_TO_FRONT() + 0.1, 0};
    return get()->getPos() + distanceToKicker.rotate(get()->getAngle());
}

}  // namespace rtt::world_new::view
