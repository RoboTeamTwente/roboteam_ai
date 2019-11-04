#include "world/world_dummy.h"

#include <iostream>

namespace rtt {

    void WorldDummy::reset() {
        blue.clear();
        yellow.clear();
        ball = Ball();
    }

    void WorldDummy::detection_callback(const proto::SSL_DetectionFrame msg) {

        blue.clear();
        yellow.clear();
        ball = Ball();

        for (auto &msg_bot : msg.robots_yellow())
        {
            rtt::Robot robot = rtt::Robot();

            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.x(), msg_bot.y());
            robot.rotate_to(msg_bot.orientation());

            yellow.push_back(robot);
        }

        for (auto &msg_bot : msg.robots_blue())
        {
            rtt::Robot robot = rtt::Robot();
            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.x(), msg_bot.y());
            robot.rotate_to(msg_bot.orientation());

            blue.push_back(robot);
        }

        if (msg.balls().size() > 0) {
            ball.set_existence(msg.balls()[0].area());
            ball.move_to(msg.balls()[0].x(), msg.balls()[0].y(), msg.balls()[0].z());
        }

    }


    proto::World WorldDummy::as_message() const {
        proto::World msg;

        for (const auto &robot : blue) {
            msg.mutable_blue()->Add(robot.as_message());
        }

        for (const auto &robot : yellow) {
            msg.mutable_yellow()->Add(robot.as_message());
        }

        auto worldBall = ball.as_message();
        msg.set_allocated_ball(&worldBall);

        return msg;
    }

}
