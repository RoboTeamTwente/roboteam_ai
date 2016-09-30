#pragma once

#include "roboteam_msgs/WorldBall.h"
#include "roboteam_utils/Position.h"


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

        
        roboteam_utils::Position get_position();
        roboteam_utils::Position get_velocity();
        uint get_area();

        roboteam_msgs::WorldBall as_message();
    };

}
