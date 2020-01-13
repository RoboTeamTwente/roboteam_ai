//
// Created by john on 1/13/20.
//

#define _EXCLUDE_QT5_
#include <cassert>
#include "../../../../roboteam_world/include/roboteam_world/world/views/ball_view.hpp"
#include "../../../include/roboteam_ai/utilities/Constants.h"

rtt::world::view::BallView::BallView(const rtt::world::ball::Ball *const _ptr) noexcept
        : _ptr{_ptr} {
    assert(_ptr && "_ptr in ball_view ctor is nullptr");
}

rtt::Vector2 rtt::world::view::BallView::getExpectedBallEndPosition() const noexcept {
    const double ballVelocitySquared = _ptr->getFilteredVelocity().length2();
    const double friction = ai::Constants::GRSIM() ? ball::SIMULATION_FRICTION : ball::REAL_FRICTION;
    return _ptr->getPos() + _ptr->getFilteredVelocity().stretchToLength(
            ballVelocitySquared / friction
    );
}

const rtt::world::ball::Ball *rtt::world::view::BallView::get() const noexcept {
    return _ptr;
}

const rtt::world::ball::Ball &rtt::world::view::BallView::operator*() const noexcept {
    return *get();
}

const rtt::world::ball::Ball *rtt::world::view::BallView::operator->() const noexcept {
    return get();
}

rtt::world::view::BallView::BallView(const rtt::world::view::BallView &old) noexcept
    : _ptr{ old.get() }
    {}

rtt::world::view::BallView &rtt::world::view::BallView::operator=(const rtt::world::view::BallView &old) noexcept {
    if (this == &old) {
        return *this;
    }
    return *this;
}
