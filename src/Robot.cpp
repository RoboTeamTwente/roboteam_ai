#include "Robot.h"

namespace rtt {

    Robot::Robot() {}


    Robot::Robot(uint id) {
        this->id = id;
    }


    Robot::Robot(uint id, float x, float y, float orientation) {
        this->id = id;
        this->x = x;
        this->y = y;
        this->orientation = orientation;
    }


    void Robot::set_id(uint id) {
        this->id = id;
    };


    void Robot::move_to(float x, float y) {
        this->x = x;
        this->y = y;
    };


    void Robot::rotate_to(float orientation) {
        this->orientation = orientation;
    };


    roboteam_world::Robot Robot::get_message() {
        roboteam_world::Robot msg;

        msg.id = id;
        msg.x = x;
        msg.y = y;
        msg.orientation = orientation;

        return msg;
    };
}
