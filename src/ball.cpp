#include "roboteam_world/ball.h"
#include <cmath>

namespace rtt {

    Ball::Ball() {
        area = 0;
        x = NAN;
        y = NAN;
        z = NAN;
    }


    void Ball::move_to(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }


    void Ball::set_area(uint area) {
        this->area = area;
    }

    
    roboteam_utils::Position Ball::get_position() {
        return roboteam_utils::Position(x, y, z);
    }
    
    roboteam_utils::Position Ball::get_velocity() {
        return roboteam_utils::Position(x_vel, y_vel, z_vel);
    }
    
    uint Ball::get_area() {
        return area;
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
