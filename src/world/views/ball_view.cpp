//
// Created by john on 1/13/20.
//

#include <cassert>
#include "../../../include/roboteam_world/world/views/ball_view.hpp"
#include "../../../include/roboteam_world/world/ball.hpp"
#include "../../../../roboteam_ai/include/roboteam_ai/utilities/Constants.h"

rtt::world::view::BallView::BallView(const rtt::world::ball::Ball *const _ptr)
        : _ptr{_ptr} {
    assert(_ptr && "_ptr in ball_view ctor is nullptr");
}

rtt::Vector2 rtt::world::view::BallView::getExpectedBallEndPosition() const {
    const double ballVelocitySquared = _ptr->getFilteredVelocity().length2();
    const double friction = ai::Constants::GRSIM() ? ball::SIMULATION_FRICTION : ball::REAL_FRICTION;
    return _ptr->getPosition() + _ptr->getFilteredVelocity().stretchToLength(
            ballVelocitySquared / friction
    );
}

const rtt::world::ball::Ball *rtt::world::view::BallView::get() {
    return _ptr;
}

const rtt::world::ball::Ball &rtt::world::view::BallView::operator*() {
    return *get();
}

const rtt::world::ball::Ball *rtt::world::view::BallView::operator->() {
    return get();
}
