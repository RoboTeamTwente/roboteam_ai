#pragma once

#include "roboteam_world/Ball.h"


namespace rtt {

    class Ball {
    private:
        uint area;
        float x;
        float y;
        float z;

    public:
        Ball();

        void move_to(float x, float y, float z);
        void set_area(uint area);

        roboteam_world::Ball as_message();
    };

}
