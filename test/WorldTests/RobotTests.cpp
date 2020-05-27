//
// Created by john on 1/22/20.
//
#include <gtest/gtest.h>
#include "world_new/Robot.hpp"
#include "world_new/views/RobotView.hpp"

namespace rtt::world_new::robot {
proto::Vector2f *getVec(float x, float y) {
    auto *data = new proto::Vector2f{};
    data->set_x(x);
    data->set_y(y);
    return data;
}

TEST(RobotAndView, test_getters) {
    proto::RobotFeedback fdbk{};
    fdbk.set_id(1);
    fdbk.set_ballsensorisworking(true);
    fdbk.set_batterylow(true);
    std::unordered_map<uint8_t, proto::RobotFeedback> feedback = {{1, fdbk}};

    proto::WorldRobot robotData{};
    robotData.set_id(1);
    robotData.set_angle(5.0);
    robotData.set_allocated_pos(getVec(5, 7));
    robotData.set_allocated_vel(getVec(3, 0));
    robotData.set_w(5.0);

    Robot data{feedback, robotData, us};

    ASSERT_EQ(data.getId(), 1);
    ASSERT_EQ(static_cast<double>(data.getAngle()), 5.0 - 2 * M_PI);
    ASSERT_EQ(data.getPos(), *getVec(5.0, 7.0));
    ASSERT_EQ(data.getVel(), *getVec(3, 0));
    ASSERT_EQ(data.getAngularVelocity(), 5.0);

    ASSERT_EQ(data.isWorkingBallSensor(), true);
    ASSERT_EQ(data.isBatteryLow(), true);

    ASSERT_EQ(view::RobotView{&data}.get(), &data);
}
}  // namespace rtt::world_new::robot