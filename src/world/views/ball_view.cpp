//
// Created by john on 1/13/20.
//

#define _EXCLUDE_QT5_
#include <cassert>
#include "world/views/ball_view.hpp"
#include "utilities/Constants.h"

namespace rtt::world_new::view {
    BallView::BallView(const rtt::world_new::ball::Ball *const _ptr) noexcept
            : _ptr{_ptr} {
        assert(_ptr && "_ptr in ball_view ctor is nullptr");
    }

    rtt::Vector2 BallView::getExpectedBallEndPosition() const noexcept {
        const double ballVelocitySquared = _ptr->getFilteredVelocity().length2();
        const double friction = ai::Constants::GRSIM() ? ball::SIMULATION_FRICTION : ball::REAL_FRICTION;
        return _ptr->getPos() + _ptr->getFilteredVelocity().stretchToLength(
                ballVelocitySquared / friction
        );
    }

    const ball::Ball *BallView::get() const noexcept {
        return _ptr;
    }

    const ball::Ball &BallView::operator*() const noexcept {
        return *get();
    }

    const ball::Ball *BallView::operator->() const noexcept {
        return get();
    }

    BallView &BallView::operator=(const BallView &old) noexcept {
        if (this == &old) {
            return *this;
        }
        return *this;
    }

    BallView &BallView::operator=(BallView &&other) noexcept {
        if (this == &other) {
            return *this;
        }
        return *this;
    }

    BallView::BallView(BallView &&other) noexcept
            : _ptr{other._ptr} {}
}