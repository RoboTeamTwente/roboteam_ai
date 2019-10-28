#include "ball.h"
#include <cmath>

namespace rtt {

    Ball::Ball() {
        existence = INVALID_AREA;
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


    void Ball::set_existence(uint existence) {
        this->existence = existence;
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
    
    uint Ball::get_existence() const {
        return existence;
    }

    proto::WorldBall Ball::as_message() const {
        proto::WorldBall msg;

        msg.set_area(existence);

        msg.mutable_pos()->set_x(x);
        msg.mutable_pos()->set_y(y);
        msg.set_z(z);

        msg.mutable_vel()->set_x(x_vel);
        msg.mutable_vel()->set_y(y_vel);
        msg.set_z_vel(z_vel);

        msg.set_visible(visible);

        return msg;
    }
}
