#include "roboteam_world/predictor.h"
#include "ros/ros.h"

namespace rtt {
        
void Predictor::discard_old_robot_data(double current_time) {
    for (auto it = ourTeamBuf.begin(); it != ourTeamBuf.end(); it++) {
        for (auto it2 = it->begin(); it2 != it->end();) {
            double timestamp = it2->first;
            if (current_time - timestamp > memory_time) {
                it2 = it->erase(it2);
            } else {
                it2++;
            }
        }
    }
    for (auto it = theirTeamBuf.begin(); it != theirTeamBuf.end(); it++) {
        for (auto it2 = it->begin(); it2 != it->end();) {
            double timestamp = it2->first;
            if (current_time - timestamp > memory_time) {
                it2 = it->erase(it2);
            } else {
                it2++;
            }
        }
    }
}   

void Predictor::discard_old_ball_data(double current_time) {
    for (auto it = ballBuf.begin(); it != ballBuf.end();) {
        double timestamp = it->first;
        if (current_time - timestamp > memory_time) {
            it = ballBuf.erase(it);
        } else {
            it++;
        }
    }
}    
    
void Predictor::update(const Robot& bot, bool our_team, double timestamp) {
    if (our_team) {
        ourTeamBuf.at(bot.get_id()).push_back({timestamp, bot});
    } else {
        theirTeamBuf.at(bot.get_id()).push_back({timestamp, bot});
    }
    discard_old_robot_data(timestamp);
}

void Predictor::update(const Ball& ball, double timestamp) {
    ballBuf.push_back({timestamp, ball});
    discard_old_ball_data(timestamp);
}    

boost::optional<roboteam_utils::Vector2> Predictor::computeBallVelocity() {
    roboteam_utils::Vector2 posDiff(0.0, 0.0);
    size_t bufferSize = ballBuf.size();
    if (bufferSize >= 2) {
        for (size_t i = 0; i < (ballBuf.size()-1); i++) {
            Ball oldBall = boost::get<Ball>(ballBuf.at(i).second);
            Ball newerBall = boost::get<Ball>(ballBuf.at(i+1).second);
            roboteam_utils::Vector2 oldBallPos(oldBall.get_position().x, oldBall.get_position().y);
            roboteam_utils::Vector2 newerBallPos(newerBall.get_position().x, newerBall.get_position().y);
            double timeDiff = ballBuf.at(i+1).first - ballBuf.at(i).first;
            posDiff = posDiff + (newerBallPos - oldBallPos).scale(1.0/timeDiff);
        }
        roboteam_utils::Vector2 ballVel = posDiff.scale(1.0/(ballBuf.size()-1));;
        return boost::optional<roboteam_utils::Vector2>(ballVel);
    }
    return boost::none;    
}

boost::optional<roboteam_utils::Vector2> Predictor::computeRobotVelocity(uint id, bool our_team) {
    if (our_team) {
        roboteam_utils::Vector2 posDiff(0.0, 0.0);
        size_t bufferSize = ourTeamBuf.at(id).size();
        if (bufferSize >= 2) {
            for (size_t i = 0; i < (ourTeamBuf.at(id).size()-1); i++) {
                Robot oldRobot = boost::get<Robot>(ourTeamBuf.at(id).at(i).second);
                Robot newerRobot = boost::get<Robot>(ourTeamBuf.at(id).at(i+1).second);
                roboteam_utils::Vector2 oldRobotPos(oldRobot.get_position().x, oldRobot.get_position().y);
                roboteam_utils::Vector2 newerRobotPos(newerRobot.get_position().x, newerRobot.get_position().y);
                double timeDiff = ourTeamBuf.at(id).at(i+1).first - ourTeamBuf.at(id).at(i).first;
                
                posDiff = posDiff + (newerRobotPos - oldRobotPos).scale(1.0/timeDiff);
            }
            roboteam_utils::Vector2 robotVel = posDiff.scale(1.0/(ourTeamBuf.at(id).size()-1));
            return boost::optional<roboteam_utils::Vector2>(robotVel);
        }
        return boost::none;  
    } else {
        roboteam_utils::Vector2 posDiff(0.0, 0.0);
        size_t bufferSize = theirTeamBuf.at(id).size();
        if (bufferSize >= 2) {
            for (size_t i = 0; i < (theirTeamBuf.at(id).size()-1); i++) {
                Robot oldRobot = boost::get<Robot>(theirTeamBuf.at(id).at(i).second);
                Robot newerRobot = boost::get<Robot>(theirTeamBuf.at(id).at(i+1).second);
                roboteam_utils::Vector2 oldBallPos(oldRobot.get_position().x, oldRobot.get_position().y);
                roboteam_utils::Vector2 newerRobotPos(newerRobot.get_position().x, newerRobot.get_position().y);
                double timeDiff = theirTeamBuf.at(id).at(i+1).first - theirTeamBuf.at(id).at(i).first;
                posDiff = posDiff + (newerRobotPos - oldBallPos).scale(1/timeDiff);
            }
            roboteam_utils::Vector2 robotVel = posDiff.scale(1.0/(theirTeamBuf.at(id).size()-1));
            return boost::optional<roboteam_utils::Vector2>(robotVel);
        }
        return boost::none; 
    }  
}

boost::optional<Position> Predictor::lookahead(const id bot_id, double seconds) const {
    return boost::optional<Position>();
}

boost::optional<Position> Predictor::lookahead_ball(double dt) const {
    return boost::optional<Position>();
}

}