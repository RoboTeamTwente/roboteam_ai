#pragma once

#include "roboteam_world/Robot.h"

namespace rtt {

    class Robot {

    private:
        uint id;
        float x;
        float y;
        float orientation;

    public:
        Robot();
        Robot(uint id);
        Robot(uint id, float x, float y, float orientation);

        uint get_id() { return id; };
        float get_x() { return x; };
        float get_y() { return y; };

        void set_id(uint id);
        void move_to(float x, float y);
        void rotate_to(float orientation);

        roboteam_world::Robot get_message();
    };

}
