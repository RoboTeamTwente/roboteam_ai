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
        // this->our_team = our_team;
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

    void Robot::update_last_detection_time(double time) {
        last_detection_time = time;
    }

    bool Robot::is_detection_old(double time, double threshold) {
        return (time - last_detection_time > threshold);
    }

    bool Robot::is_detection_from_future(double time) {
        return (time < last_detection_time);
    }

    Position Robot::get_position() const {
        return Position(x, y, angle);
    }

    Position Robot::get_velocity() const {
        return Position(x_vel, y_vel, w);
    }

    uint Robot::get_id() const {
        return id;
    }

    roboteam_msgs::WorldRobot Robot::as_message() const {
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
