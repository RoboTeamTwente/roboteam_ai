//
// Created by john on 1/22/20.
//
#include <gtest/gtest.h>
#include "world_new/Ball.hpp"
#include "world_new/views/BallView.hpp"

namespace rtt::world_new::ball {
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
    Ball data{protoData};

    EXPECT_EQ(data.getVelocity(), *getVec(10.0, 3.0));
    EXPECT_EQ(data.getPos(), *getVec(10.0, 3.0));  // yeye memory leak doesn't rlly matter
    EXPECT_EQ(data.isVisible(), true);
    EXPECT_EQ(0, 0);

    view::BallView _view{&data};
    EXPECT_EQ(data.getVelocity(), _view->getVelocity());
    EXPECT_EQ(data.getPos(), _view->getPos());
    EXPECT_EQ(data.isVisible(), _view->isVisible());

    EXPECT_EQ((bool)_view, true);
    EXPECT_EQ(_view.get(), &data);
}
}  // namespace rtt::world_new::ball