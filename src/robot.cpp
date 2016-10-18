#include "roboteam_world/robot.h"
#include <cmath>

namespace rtt {

    Robot::Robot() : Robot(INVALID_ROBOT_ID) {
    }


    Robot::Robot(uint id) : Robot(id, NAN, NAN, NAN) {
    }


    Robot::Robot(uint id, float x, float y, float angle) {
        this->id = id;
        this->x = x;
        this->y = y;
        this->angle = angle;
    }


    void Robot::set_id(uint id) {
        this->id = id;
    };


    void Robot::move_to(float x, float y) {
        this->x = x;
        this->y = y;
    };


    void Robot::rotate_to(float angle) {
        this->angle = angle;
    };

    void Robot::set_vel(float x_vel, float y_vel, float w) {
        this->x_vel = x_vel;
        this->y_vel = y_vel;
        this->w = w;
    }

    roboteam_utils::Position Robot::get_position() {
        return roboteam_utils::Position(x, y, angle);
    }
    
    roboteam_utils::Position Robot::get_velocity() {
        return roboteam_utils::Position(x_vel, y_vel, w);
    }
    
    uint Robot::get_id() {
        return id;
    }

    roboteam_msgs::WorldRobot Robot::as_message() {
        roboteam_msgs::WorldRobot msg;

        msg.id = id;

        msg.pos.x = x;
        msg.pos.y = y;
        msg.angle = angle;

        msg.vel.x = x_vel;
        msg.vel.y = y_vel;
        msg.w = w;

        return msg;
    };
}
