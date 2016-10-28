#include "roboteam_world/predictor.h"

namespace rtt {
        
void Predictor::discard_old_data(double current_time) {
    for (auto it = buf.begin(); it != buf.end(); it++) {
        for (auto it2 = it->second.begin(); it2 != it->second.end();) {
            double timestamp = it->first;
            if (current_time - timestamp > memory_time) {
                it2 = it->second.erase(it2);
            } else {
                it2++;
            }
        }
    }
}    
    
void Predictor::update(const Robot& bot, double timestamp) {
    if (buf.find(bot.get_id()) == buf.end()) {
        buf[bot.get_id()] = std::vector<std::pair<double, Entity>>();
    }
    buf.at(bot.get_id()).push_back({timestamp, bot});
    discard_old_data(timestamp);
}

void Predictor::update(const Ball& ball, double timestamp) {
    if (buf.find(BALL_ID) == buf.end()) {
        buf[BALL_ID] = std::vector<std::pair<double, Entity>>();
    }
    buf.at(BALL_ID).push_back({timestamp, ball});
    discard_old_data(timestamp);
}    

boost::optional<Position> Predictor::last_pos(const Robot& bot) const {
    if (buf.find(bot.get_id()) == buf.end()) {
        return boost::optional<Position>();
    }
    const auto vec = buf.at(bot.get_id());
    auto it = vec.end();
    it--;
    return boost::optional<Position>(boost::get<Robot>(it->second).get_position());
}

boost::optional<Position> Predictor::last_pos(const Ball& ball) const {
    if (buf.find(BALL_ID) == buf.end()) {
        return boost::optional<Position>();
    }
    const auto vec = buf.at(BALL_ID);
    auto it = vec.end();
    it--;
    return boost::optional<Position>(boost::get<Ball>(it->second).get_position());
} 

boost::optional<Position> Predictor::lookahead(const id bot_id, double dt) const {
    return boost::optional<Position>();
}

boost::optional<Position> Predictor::lookahead_ball(double dt) const {
    return boost::optional<Position>();
}

}