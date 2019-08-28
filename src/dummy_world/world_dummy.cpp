#include "roboteam_world/world/world_dummy.h"

#include <iostream>

namespace rtt {

    void WorldDummy::reset() {
        us.clear();
        them.clear();
        ball = Ball();
    }

    void WorldDummy::detection_callback(const roboteam_proto::DetectionFrame msg) {

        us.clear();
        them.clear();
        ball = Ball();

        for (auto &msg_bot : msg.us())
        {
            rtt::Robot robot = rtt::Robot();

            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.pos().x(), msg_bot.pos().y());
            robot.rotate_to(msg_bot.orientation());

            us.push_back(robot);
        }

        for (auto &msg_bot : msg.them())
        {
            rtt::Robot robot = rtt::Robot();
            robot.set_id(msg_bot.robot_id());
            robot.move_to(msg_bot.pos().x(), msg_bot.pos().y());
            robot.rotate_to(msg_bot.orientation());

            them.push_back(robot);
        }

        if (msg.balls().size() > 0) {
            ball.set_existence(msg.balls()[0].existence());
            ball.move_to(msg.balls()[0].pos().x(), msg.balls()[0].pos().y(), msg.balls()[0].z());
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
