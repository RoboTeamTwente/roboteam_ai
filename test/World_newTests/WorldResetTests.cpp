//
// Created by john on 3/2/20.
//

#include <gtest/gtest.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include <test/helpers/WorldHelper.h>

TEST(World_newTest, ResetWorldTest) {
    namespace w_n = rtt::world_new;
    auto _ball = w_n::World::instance()->getBall();
    _ball-
}