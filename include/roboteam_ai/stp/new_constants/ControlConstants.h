//
// Created by jessevw on 25.03.20.
//

#ifndef RTT_CONTROLCONSTANTS_H
#define RTT_CONTROLCONSTANTS_H

namespace rtt::ai::stp::control_constants {
    // Kick and Chip constants
    extern const double MAX_CHIP_SPEED;
    extern const double MAX_KICK_POWER;
    extern const double MIN_KICK_POWER;
    extern const double MAX_CHIP_POWER;
    extern const double MIN_CHIP_POWER;
    extern const double DEFAULT_KICK_POWER;
    extern const double MAX_POWER_KICK_DISTANCE;
    extern const double MAX_POWER_CHIP_DISTANCE;

    // Team constants TODO: Maybe this should be in a different constants file
    inline constexpr size_t MAX_ROBOT_COUNT = 11;

    //
    extern const double BALL_STILL_VEL;

    // RobotCommand limits
    extern const double MAX_VEL_CMD;
    extern const double MAX_DRIBBLER_CMD;

    // GTP Constants
    extern const double GO_TO_POS_ERROR_MARGIN;
    extern const double GO_TO_POS_ANGLE_ERROR_MARGIN;

    // Robot physical constants
    extern const double BALL_RADIUS;
    extern const double ROBOT_RADIUS;
    extern const double ROBOT_RADIUS_MAX;
    extern const double CENTER_TO_FRONT;
    extern const double DRIBBLER_ANGLE_OFFSET;
    extern const double FRONT_LENGTH;
}

#endif //RTT_CONTROLCONSTANTS_H
