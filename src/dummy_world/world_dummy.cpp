#include "roboteam_world/world/world_dummy.h"

#include <iostream>

namespace rtt {

    void WorldDummy::reset() {
        us.clear();
        them.clear();
        ball = Ball();
    }

    void WorldDummy::detection_callback(const roboteam_msgs::DetectionFrame msg) {

        us.clear();
        them.clear();
        ball = Ball();

        std::vector<roboteam_msgs::DetectionRobot> yellow = msg.us;
        std::vector<roboteam_msgs::DetectionRobot> blue = msg.them;

        std::vector<roboteam_msgs::DetectionBall> balls = msg.balls;

        for (auto &msg_bot : yellow)
        {
            rtt::Robot robot = rtt::Robot();

            robot.set_id(msg_bot.robot_id);
            robot.move_to(msg_bot.pos.x, msg_bot.pos.y);
            robot.rotate_to(msg_bot.orientation);

            us.push_back(robot);
        }

        for (auto &msg_bot : blue)
        {
            rtt::Robot robot = rtt::Robot();
            robot.set_id(msg_bot.robot_id);
            robot.move_to(msg_bot.pos.x, msg_bot.pos.y);
            robot.rotate_to(msg_bot.orientation);

            them.push_back(robot);
        }

        if (balls.size() > 0) {
            ball.set_area(balls[0].area);
            ball.move_to(balls[0].pos.x, balls[0].pos.y, balls[0].z);
        }

    }


    roboteam_msgs::World WorldDummy::as_message() const {
        roboteam_msgs::World msg;

        for (const auto &robot : them) {
            msg.them.push_back(robot.as_message());
        }

        for (const auto &robot : us) {
            msg.us.push_back(robot.as_message());
        }

        msg.ball = ball.as_message();

        return msg;
    }

}
