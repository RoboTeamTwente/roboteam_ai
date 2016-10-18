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

        for (uint i = 0; i < yellow.size(); ++i)
        {
            rtt::Robot robot = rtt::Robot();

            robot.set_id(yellow[i].robot_id);
            robot.move_to(yellow[i].pos.x, yellow[i].pos.y);
            robot.rotate_to(yellow[i].orientation);

            us.push_back(robot);
        }

        for (uint i = 0; i < blue.size(); ++i)
        {
            rtt::Robot robot = rtt::Robot();
            robot.set_id(blue[i].robot_id);
            robot.move_to(blue[i].pos.x, blue[i].pos.y);
            robot.rotate_to(blue[i].orientation);

            them.push_back(robot);
        }

        if (balls.size() > 0) {
            ball.set_area(balls[0].area);
            ball.move_to(balls[0].pos.x, balls[0].pos.y, balls[0].z);
        }

    }


    roboteam_msgs::World WorldDummy::as_message() {
        roboteam_msgs::World msg;

        for (std::vector<rtt::Robot>::iterator it = them.begin(); it != them.end(); it++) {
            msg.them.push_back(it->as_message());
        }

        for (std::vector<rtt::Robot>::iterator it = us.begin(); it != us.end(); it++) {
            msg.us.push_back(it->as_message());
        }

        msg.ball = ball.as_message();

        return msg;
    }

}
