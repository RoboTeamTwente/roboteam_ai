#include "roboteam_world/predictor.h"

namespace rtt {

inline Position pos_of(const Robot& bot) {
    return Position(bot.x, bot.y, bot.angle);
}

inline Position pos_of(const Ball& ball) {
    return Position(ball.x, ball.y, -1.0);
}
        
void Predictor::discard_old_data(double current_time) {
    for (auto it = buf.begin(); it != buf.end(); it++) {
        for (auto it2 = it->begin(); it2 != it->end();) {
            double timestamp = it->first;
            if (current_time - timestamp > memory_time) {
                it2 = it->erase(it2);
            } else {
                it2++;
            }
        }
    }
}    
    
void Predictor::update(const Robot& bot, double timestamp) {
    if (buf.find(bot.id) == buf.end()) {
        buf[bot.id] = std::vector<std::pair<double, Entity>>();
    }
    buf.at(bot.id).push_back({bot, timestamp});
    discard_old_data(timestamp);
}

void Predictor::update(const Ball& ball, double timestamp) {
    if (buf.find(BALL_ID) == buf.end()) {
        buf[BALL_ID] = std::vector<std::pair<double, Entity>>();
    }
    buf.at(BALL_ID).push_back({ball, timestamp});
    discard_old_data(timestamp);
}    

boost::optional<Position> Predictor::last_pos(const Robot& bot) {
    if (buf.find(bot.id) == buf.end()) {
        return boost::optional<Position>();
    }
    return boost::optional<Position>(pos_of(boost::get<Robot>(buf.at(bot.id).second)));
}

boost::optional<Position> Predictor::last_pos(const Ball& ball) {
    if (buf.find(BALL_ID) == buf.end()) {
        return boost::optional<Position>();
    }
    return boost::optional<Position>(pos_of(boost::get<Ball>(buf.at(BALL_ID).second)));
} 

boost::optional<Position> Predictor::lookahead(const id bot_id, double dt) const {
    return boost::optional<Position>();
}

boost::optional<Position> Predictor::lookahead_ball(double dt) const;
    return boost::optional<Position>();
}