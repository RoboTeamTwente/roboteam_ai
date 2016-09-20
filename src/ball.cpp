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


    roboteam_world::Ball Ball::as_message() {
        roboteam_world::Ball msg;

        msg.area = area;
        msg.x = x;
        msg.y = y;
        msg.z = z;

        return msg;
    }
}
