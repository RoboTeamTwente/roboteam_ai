//
// Created by john on 1/22/20.
//
#include <gtest/gtest.h>
#include "include/roboteam_ai/world/Robot.hpp"
#include "world/views/RobotView.hpp"

namespace rtt::world::robot {
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
    ASSERT_EQ(data.getAngle(), Angle(5.0));
    ASSERT_EQ(data.getPos(), Vector2(5.0, 7.0));
    ASSERT_EQ(data.getVel(), Vector2(3, 0));
    ASSERT_EQ(data.getAngularVelocity(), 5.0);

    ASSERT_EQ(data.isWorkingBallSensor(), true);
    ASSERT_EQ(data.isBatteryLow(), true);

    ASSERT_EQ(view::RobotView{&data}.get(), &data);
}
}  // namespace rtt::world::robot