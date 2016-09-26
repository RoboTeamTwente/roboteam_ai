#include "robot.h"

namespace rtt {

    Robot::Robot() {}


    Robot::Robot(uint id) {
        this->id = id;
    }


    Robot::Robot(uint id, float x, float y, float w) {
        this->id = id;
        this->x = x;
        this->y = y;
        this->w = w;
    }


    void Robot::set_id(uint id) {
        this->id = id;
    };


    void Robot::move_to(float x, float y) {
        this->x = x;
        this->y = y;
    };


    void Robot::rotate_to(float w) {
        this->w = w;
    };


    roboteam_msgs::WorldRobot Robot::as_message() {
        roboteam_msgs::WorldRobot msg;

        msg.id = id;

        msg.pos.x = x;
        msg.pos.y = y;
        msg.w = w;

        msg.vel.x = x_vel;
        msg.vel.y = y_vel;
        msg.w_vel = w_vel;

        return msg;
    };
}
