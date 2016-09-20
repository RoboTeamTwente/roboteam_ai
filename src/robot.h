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

        void set_id(uint id);
        void move_to(float x, float y);
        void rotate_to(float orientation);

        roboteam_world::Robot as_message();
    };

}
