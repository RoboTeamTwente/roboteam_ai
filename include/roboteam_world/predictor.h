#pragma once

#include <array>
#include <map>
#include <inttypes.h>
#include <utility>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include "roboteam_utils/Position.h"

#define BALL_ID 123456789

namespace rtt {

const int NUM_ROBOTS = 15;

// typedef uint32_t id;
typedef std::array<std::vector<std::pair<double, Robot>>, NUM_ROBOTS> RobotBuffer;
typedef std::vector<std::pair<double, Ball>> BallBuffer;

class Predictor {
    private:
    // TODO: make better comments
    // The robots we have on the field
    RobotBuffer ourTeamBuffer;
    // The robots the opponents have on the field
    RobotBuffer theirTeamBuffer;
    BallBuffer ballBuf;
    double memory_time;
    /// If a robot is not visible for some time remove it, also clean the timestamps for the current robots
    void discard_old_robot_data(double current_time);
    /// Clean the ball data if the ball is not visible for some time
    void discard_old_ball_data(double current_time);

    public:
    Predictor(double memory_time = 0.1 /*seconds*/) : memory_time(memory_time) {}
    /// Update the buffer for robots with time stamps
    void update(const Robot& robot, bool our_team, double timestamp);
    /// Update the buffer for the ball(s) with time stamps
    void update(const Ball& ball, double timestamp);
    /// Compute ball velocity from the ballBuf data
    boost::optional<Position> computeBallVelocity();
    /// Compute robot velocity from ourTeamBuffer and theirTeamBuffer data
    boost::optional<Position> computeRobotVelocity(uint id, bool our_team);

    // TODO: fix never used
    boost::optional<Position> lookahead(const uint bot_id, bool our_team, double dt) const;
    boost::optional<Position> lookahead_ball(double dt) const;
};

}
