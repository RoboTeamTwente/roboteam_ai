#include "roboteam_world/ball.h"

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

        msg.pos.x = x;
        msg.pos.y = y;
        msg.z = z;

        msg.vel.x;
        msg.vel.y;
        msg.z_vel;

        return msg;
    }
}
