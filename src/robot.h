#pragma once

#include "roboteam_world/Robot.h"

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

        roboteam_world::Robot as_message();
    };

}
