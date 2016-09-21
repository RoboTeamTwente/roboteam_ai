#include "ball.h"

namespace rtt {

    Ball::Ball() {

    }


    void Ball::move_to(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }


    void Ball::set_area(uint area) {
        this->area = area;
    }


    roboteam_msgs::WorldBall Ball::as_message() {
        roboteam_msgs::WorldBall msg;

        msg.area = area;

        msg.x = x;
        msg.y = y;
        msg.z = z;

        msg.x_vel;
        msg.y_vel;
        msg.z_vel;

        return msg;
    }
}
