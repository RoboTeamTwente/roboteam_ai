#pragma once

#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

    class Robot {

    private:
        uint id;
        float x;
        float y;
        float w;

        float x_vel;
        float y_vel;
        float w_vel;

    public:
        Robot();
        Robot(uint id);
        Robot(uint id, float x, float y, float w);

        void set_id(uint id);
        void move_to(float x, float y);
        void rotate_to(float w);
        void set_vel(float x_vel, float y_vel, float w_vel);

        roboteam_msgs::WorldRobot as_message();
    };

}
