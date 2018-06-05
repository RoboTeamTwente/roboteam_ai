#include "roboteam_world/ball.h"
#include <cmath>

namespace rtt {

    Ball::Ball() {
        area = INVALID_AREA;
        x = NAN;
        y = NAN;
        z = NAN;
    }


    void Ball::move_to(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void Ball::set_velocity(float x_vel, float y_vel) {
        this->x_vel = x_vel;
        this->y_vel = y_vel;
    }


    void Ball::set_area(uint area) {
        this->area = area;
    }

    void Ball::set_visible(bool visible) {
        this->visible = visible;
    }
    
    Position Ball::get_position() const {
        return Position(x, y, z);
    }
    
    Position Ball::get_velocity() const {
        return Position(x_vel, y_vel, z_vel);
    }
    
    uint Ball::get_area() const {
        return area;
    }

    roboteam_msgs::WorldBall Ball::as_message() const {
        roboteam_msgs::WorldBall msg;

        msg.area = area;

        msg.pos.x = x;
        msg.pos.y = y;
        msg.z = z;

        msg.vel.x = x_vel;
        msg.vel.y = y_vel;
        msg.z_vel = z_vel;

        msg.visible = visible;

        return msg;
    }
}
