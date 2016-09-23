#include "world_dummy.h"

#include <iostream>


namespace rtt {

    void WorldDummy::reset() {
        robots_yellow.clear();
        robots_blue.clear();
        ball = Ball();
    }


    void WorldDummy::detection_callback(const roboteam_msgs::DetectionFrame msg) {

        if (msg.camera_id < config.num_cams()) {

            robots_yellow.clear();
            robots_blue.clear();
            ball = Ball();

            std::vector<roboteam_msgs::DetectionRobot> yellow = msg.robots_yellow;
            std::vector<roboteam_msgs::DetectionRobot> blue = msg.robots_blue;

            std::vector<roboteam_msgs::DetectionBall> balls = msg.balls;

            for (int i = 0; i < yellow.size(); ++i)
            {
                rtt::Robot robot = rtt::Robot();

                robot.set_id(yellow[i].robot_id);
                robot.move_to(yellow[i].x, yellow[i].y);
                robot.rotate_to(yellow[i].orientation);

                robots_yellow.push_back(robot);
            }

            for (int i = 0; i < blue.size(); ++i)
            {
                rtt::Robot robot = rtt::Robot();
                robot.set_id(blue[i].robot_id);
                robot.move_to(blue[i].x, blue[i].y);
                robot.rotate_to(blue[i].orientation);

                robots_blue.push_back(robot);
            }

            if (balls.size() > 0) {
                ball.set_area(balls[0].area);
                ball.move_to(balls[0].x, balls[0].y, balls[0].z);
            }

        }

    }


    roboteam_msgs::World WorldDummy::as_message() {
        roboteam_msgs::World msg;

        for (std::vector<rtt::Robot>::iterator it = robots_blue.begin(); it != robots_blue.end(); it++) {
            msg.robots_blue.push_back(it->as_message());
        }

        for (std::vector<rtt::Robot>::iterator it = robots_yellow.begin(); it != robots_yellow.end(); it++) {
            msg.robots_yellow.push_back(it->as_message());
        }

        msg.ball = ball.as_message();

        return msg;
    }

}
