#include "roboteam_world/world/world_dummy.h"

#include <iostream>

namespace rtt {

    void WorldDummy::reset() {
        us.clear();
        them.clear();
        ball = Ball();
    }

    void WorldDummy::detection_callback(const roboteam_proto::SSL_DetectionFrame msg) {

        us.clear();
        them.clear();
        ball = Ball();

        for (auto &msg_bot : msg.robots_yellow())
        {
            rtt::Robot robot = rtt::Robot();

            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.x(), msg_bot.y());
            robot.rotate_to(msg_bot.orientation());

            us.push_back(robot);
        }

        for (auto &msg_bot : msg.robots_blue())
        {
            rtt::Robot robot = rtt::Robot();
            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.x(), msg_bot.y());
            robot.rotate_to(msg_bot.orientation());

            them.push_back(robot);
        }

        if (msg.balls().size() > 0) {
            ball.set_existence(msg.balls()[0].area());
            ball.move_to(msg.balls()[0].x(), msg.balls()[0].y(), msg.balls()[0].z());
        }

    }


    roboteam_proto::World WorldDummy::as_message() const {
        roboteam_proto::World msg;

        for (const auto &robot : them) {
            msg.mutable_them()->Add(robot.as_message());
        }

        for (const auto &robot : us) {
            msg.mutable_us()->Add(robot.as_message());
        }

        auto worldBall = ball.as_message();
        msg.set_allocated_ball(&worldBall);

        return msg;
    }

}
