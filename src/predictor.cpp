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

boost::optional<Position> Predictor::computeBallVelocity() {
    Position posDiff(0.0, 0.0, 0.0);
    size_t bufferSize = ballBuf.size();
    if (bufferSize >= 2) {
        // for (size_t i = 0; i < (ballBuf.size()-1); i++) {
        for (size_t i = 0; i < 1; i++) {
            Ball oldBall = boost::get<Ball>(ballBuf.at(i).second);
            Ball newerBall = boost::get<Ball>(ballBuf.at(i+1).second);
            Position oldBallPos = oldBall.get_position();
            Position newerBallPos = newerBall.get_position();
            double timeDiff = ballBuf.at(i+1).first - ballBuf.at(i).first;
            Position thisPosDiff = newerBallPos - oldBallPos;
            // ROS_INFO_STREAM("thisPosDiff: " << sqrt(thisPosDiff.x*thisPosDiff.x + thisPosDiff.y*thisPosDiff.y));
            posDiff = posDiff + thisPosDiff.scale(1.0/timeDiff);
        }

        Position ballVel = posDiff.scale(1.0/(ballBuf.size()-1));
        double ballSpeed = sqrt(ballVel.x*ballVel.x + ballVel.y*ballVel.y);
        // if (ballSpeed > 0.1) {
            // ROS_INFO_STREAM("ballSpeed: " << ballSpeed);
        // }
        
        return boost::optional<Position>(ballVel);
    }
    return boost::none;
}

boost::optional<Position> Predictor::computeRobotVelocity(uint id, bool our_team) {
    if (our_team) {
        Position posDiff(0.0, 0.0, 0.0);
        size_t bufferSize = ourTeamBuf.at(id).size();
        if (bufferSize >= 2) {
            for (size_t i = 0; i < (ourTeamBuf.at(id).size()-1); i++) {
                Robot oldRobot = boost::get<Robot>(ourTeamBuf.at(id).at(i).second);
                Robot newerRobot = boost::get<Robot>(ourTeamBuf.at(id).at(i+1).second);
                Position oldRobotPos = oldRobot.get_position();
                Position newerRobotPos = newerRobot.get_position();
                double timeDiff = ourTeamBuf.at(id).at(i+1).first - ourTeamBuf.at(id).at(i).first;

                posDiff = posDiff + (newerRobotPos - oldRobotPos).scale(1.0/timeDiff);
            }
            Position robotVel = posDiff.scale(1.0/(ourTeamBuf.at(id).size()-1));

            return boost::optional<Position>(robotVel);
        }
        return boost::none;
    } else {
        Position posDiff(0.0, 0.0, 0.0);
        size_t bufferSize = theirTeamBuf.at(id).size();
        if (bufferSize >= 2) {
            for (size_t i = 0; i < (theirTeamBuf.at(id).size()-1); i++) {
                Robot oldRobot = boost::get<Robot>(theirTeamBuf.at(id).at(i).second);
                Robot newerRobot = boost::get<Robot>(theirTeamBuf.at(id).at(i+1).second);
                Position oldBallPos = oldRobot.get_position();
                Position newerRobotPos = newerRobot.get_position();
                double timeDiff = theirTeamBuf.at(id).at(i+1).first - theirTeamBuf.at(id).at(i).first;
                posDiff = posDiff + (newerRobotPos - oldBallPos).scale(1/timeDiff);
            }
            Position robotVel = posDiff.scale(1.0/(theirTeamBuf.at(id).size()-1));
            
            return boost::optional<Position>(robotVel);
        }
        return boost::none;
    }
}

boost::optional<Position> Predictor::lookahead(const uint bot_id, bool our_team, double seconds) const {
    Robot robot;
    if (our_team) {
        robot = boost::get<Robot>(ourTeamBuf.at(bot_id).at(ourTeamBuf.at(bot_id).size()-1).second);
    } else {
        robot = boost::get<Robot>(theirTeamBuf.at(bot_id).at(theirTeamBuf.at(bot_id).size()-1).second);
    }
    Position latestPos = robot.get_position();
    Position latestVel = robot.get_velocity();

    Position predictedPos = latestPos + latestVel*seconds;
    return boost::optional<Position>(predictedPos);
}

boost::optional<Position> Predictor::lookahead_ball(double seconds) const {
    Ball ball = boost::get<Ball>(ballBuf.at(ballBuf.size()-1).second);

    Position latestPos = ball.get_position();
    Position latestVel = ball.get_velocity();

    Position predictedPos = latestPos + latestVel*seconds;
    return boost::optional<Position>(predictedPos);
}

} // rtt
