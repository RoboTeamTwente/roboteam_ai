#pragma once

#include "roboteam_msgs/WorldBall.h"


namespace rtt {

    class Ball {
    private:
        uint area;
        float x;
        float y;
        float z;

        float x_vel;
        float y_vel;
        float z_vel;

    public:
        Ball();

        void move_to(float x, float y, float z);
        void set_area(uint area);

        roboteam_msgs::WorldBall as_message();
    };

}
