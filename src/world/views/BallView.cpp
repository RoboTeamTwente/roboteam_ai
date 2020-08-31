//
// Created by john on 1/13/20.
//

#include "world/views/BallView.hpp"

namespace rtt::world::view {
BallView::BallView(const rtt::world::ball::Ball *const _ptr) noexcept : _ptr{_ptr} {}

const ball::Ball *BallView::get() const noexcept { return _ptr; }

const ball::Ball &BallView::operator*() const noexcept { return *get(); }

const ball::Ball *BallView::operator->() const noexcept { return get(); }

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

BallView::BallView(BallView &&other) noexcept : _ptr{other._ptr} {}
}  // namespace rtt::world::view
rtt::world::view::BallView::operator bool() const noexcept { return get() != nullptr; }
