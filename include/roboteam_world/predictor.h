#pragma once

#include <vector>
#include <map>
#include <inttypes.h>
#include <utility>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include "roboteam_utils/Position.h"

#define BALL_ID 1234567654321

use Position = roboteam_utils::Position;

namespace rtt {

typedef uint32_t id;
typedef boost::variant<Robot, Ball> Entity;
typedef std::map<id, std::vector<std::pair<double, Entity>>> Buffer;

class Predictor {
    private:
    Buffer buf;
    double memory_time;
    void discard_old_data(double current_time);
    
    public:
    Predictor(double memory_time = 2.0 /*seconds*/) : memory_time(memory_time) {}
    void update(const Robot& bot, double timestamp);
    void update(const Ball& ball, double timestamp);
    boost::optional<Position> last_pos(const Robot& bot) const;
    boost::optional<Position> last_pos(const Ball& ball) const;
    boost::optional<Position> lookahead(const id bot_id, double dt) const;
    boost::optional<Position> lookahead_ball(double dt) const;
};
    
}