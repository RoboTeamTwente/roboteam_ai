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
    RobotBuffer ourTeamBuf;
    RobotBuffer theirTeamBuf;
    BallBuffer ballBuf;
    double memory_time;
    void discard_old_robot_data(double current_time);
    void discard_old_ball_data(double current_time);

    public:
    Predictor(double memory_time = 2.0 /*seconds*/) : memory_time(memory_time) {}
    void update(const Robot& bot, bool our_team, double timestamp);
    void update(const Ball& ball, double timestamp);
    boost::optional<Position> computeBallVelocity();
    boost::optional<Position> computeRobotVelocity(uint id, bool our_team);
    boost::optional<Position> lookahead(const uint bot_id, bool our_team, double dt) const;
    boost::optional<Position> lookahead_ball(double dt) const;
};

}
