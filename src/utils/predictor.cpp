#include "roboteam_world/predictor.h"
#include "ros/ros.h"
#include "roboteam_utils/Math.h"
//TODO: include acceleration calculator
namespace rtt {

void Predictor::discard_old_robot_data(double current_time) {
    for (auto &robotInBuffer : ourTeamBuffer) {
        for (auto robotData = robotInBuffer.begin(); robotData != robotInBuffer.end();) {
            double timestamp = robotData->first;
            if (current_time - timestamp > memory_time) {
                robotData = robotInBuffer.erase(robotData);
            } else {
                robotData++;
            }
        }
    }
    for (auto &opponentInBuffer : theirTeamBuffer) {
        for (auto robotData = opponentInBuffer.begin(); robotData != opponentInBuffer.end();) {
            double timestamp = robotData->first;
            if (current_time - timestamp > memory_time) {
                robotData = opponentInBuffer.erase(robotData);
            } else {
                robotData++;
            }
        }
    }
}

void Predictor::discard_old_ball_data(double current_time) {
    for (auto currentBall = ballBuf.begin(); currentBall != ballBuf.end();) {
        double timestamp = currentBall->first;
        if (current_time - timestamp > memory_time) {
            currentBall = ballBuf.erase(currentBall);
        } else {
            currentBall++;
        }
    }
}

void Predictor::update(const Robot& robot, bool our_team, double timestamp) {
    if (our_team) {
        ourTeamBuffer.at(robot.get_id()).push_back({timestamp, robot});
    } else {
        theirTeamBuffer.at(robot.get_id()).push_back({timestamp, robot});
    }
    discard_old_robot_data(timestamp);
}

void Predictor::update(const Ball& ball, double timestamp) {
    ballBuf.push_back({timestamp, ball});
    discard_old_ball_data(timestamp);
}

boost::optional<Position> Predictor::computeBallVelocity() {
    // Position posDiff(0.0, 0.0, 0.0);
    size_t bufferSize = ballBuf.size();
    if (bufferSize >= 2) {
        Ball oldBall = boost::get<Ball>(ballBuf.at(0).second);
        Ball newerBall = boost::get<Ball>(ballBuf.at(bufferSize-1).second);
        Position oldBallPos = oldBall.get_position();
        Position newerBallPos = newerBall.get_position();

        Position posDiff = newerBallPos - oldBallPos;
        double timeDiff = ballBuf.at(bufferSize-1).first - ballBuf.at(0).first;
        Position ballVel = posDiff.scale(1.0/timeDiff);
        
        return boost::optional<Position>(ballVel);
    }
    return boost::none;
}

boost::optional<Position> Predictor::computeRobotVelocity(uint id, bool our_team) {
    if (our_team) {
        size_t bufferSize = ourTeamBuffer.at(id).size();
        if (bufferSize >= 2) {
            // Take only the first and last entry in the buffer for computing average velocity
            Robot oldRobot = boost::get<Robot>(ourTeamBuffer.at(id).at(0).second);
            Robot newerRobot = boost::get<Robot>(ourTeamBuffer.at(id).at(bufferSize-1).second);
            Position oldRobotPos = oldRobot.get_position();
            Position newerRobotPos = newerRobot.get_position();

            Position posDiff = newerRobotPos - oldRobotPos;
            posDiff.rot = cleanAngle(posDiff.rot); // for where my angle flips from pi to -pi
            double timeDiff = ourTeamBuffer.at(id).at(bufferSize-1).first - ourTeamBuffer.at(id).at(0).first;

            Position robotVel = posDiff.scale(1.0/timeDiff);

            return boost::optional<Position>(robotVel);
        }
        return boost::none;
    } else {
        size_t bufferSize = theirTeamBuffer.at(id).size();
        if (bufferSize >= 2) {
            // Take only the first and last entry in the buffer for computing average velocity
            Robot oldRobot = boost::get<Robot>(theirTeamBuffer.at(id).at(0).second);
            Robot newerRobot = boost::get<Robot>(theirTeamBuffer.at(id).at(bufferSize-1).second);
            Position oldRobotPos = oldRobot.get_position();
            Position newerRobotPos = newerRobot.get_position();

            Position posDiff = newerRobotPos - oldRobotPos;
            posDiff.rot = cleanAngle(posDiff.rot); // for where my angle flips from pi to -pi
            double timeDiff = theirTeamBuffer.at(id).at(bufferSize-1).first - theirTeamBuffer.at(id).at(0).first;

            Position robotVel = posDiff.scale(1.0/timeDiff);

            return boost::optional<Position>(robotVel);
        }
        return boost::none;
    }
}

} // rtt
