#pragma once

#include "roboteam_msgs/WorldBall.h"
#include "roboteam_utils/Position.h"

#define INVALID_AREA 99999

namespace rtt {

    class Ball {
    private:
        uint existence;
        float x;
        float y;
        float z;

        float x_vel;
        float y_vel;
        float z_vel;
        
        bool visible;

    public:
        Ball();

        void move_to(float x, float y, float z);
        void set_velocity(float x_vel, float y_vel);
        void set_existence(uint existence);
        void set_visible(bool visible);

        Position get_position() const;
        Position get_velocity() const;
        uint get_existence() const;

        roboteam_msgs::WorldBall as_message() const;
    };

}
