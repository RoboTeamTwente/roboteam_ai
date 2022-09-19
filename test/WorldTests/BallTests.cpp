//
// Created by john on 1/22/20.
//
#include <gtest/gtest.h>

#include "world/Ball.hpp"
#include "world/World.hpp"
#include "world/views/BallView.hpp"

namespace rtt::world::ball {
proto::Vector2f *getVec(float x, float y) {
    auto *data = new proto::Vector2f{};
    data->set_x(x);
    data->set_y(y);
    return data;
}

TEST(BallAndView, test_getters) {
    proto::WorldBall protoData;
    protoData.set_allocated_pos(getVec(10.0, 3.0));
    protoData.set_allocated_vel(getVec(10.0, 3.0));
    protoData.set_visible(true);
    auto const &[_, world] = rtt::world::World::instance();
    Ball data{protoData, world};

    EXPECT_EQ(data.velocity, Vector2(10.0, 3.0));
    EXPECT_EQ(data.position, Vector2(10.0, 3.0));  // yeye memory leak doesn't rlly matter
    EXPECT_EQ(data.visible, true);
    EXPECT_EQ(0, 0);

    view::BallView _view{&data};
    EXPECT_EQ(data.velocity, _view->velocity);
    EXPECT_EQ(data.position, _view->position);
    EXPECT_EQ(data.visible, _view->visible);

    EXPECT_EQ((bool)_view, true);
    EXPECT_EQ(_view.get(), &data);
}
}  // namespace rtt::world::ball