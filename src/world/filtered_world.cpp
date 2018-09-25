#include <utility>

#include "roboteam_world/world/filtered_world.h"
#include <vector>
#include "roboteam_utils/Vector2.h"

namespace rtt {

    FilteredWorld::FilteredWorld(Predictor predictor) : fresh{false} {
        reset();
        this->predictor = std::move(predictor);
    }

    void FilteredWorld::reset() {

        // Reset all vectors.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();

        // These are used for the final world state
        robots_blue_world.clear();
        robots_yellow_world.clear();
        ball_world = rtt::Ball();

        // Initialize the input buffers.
        robots_blue_buffer = RobotMultiCamBuffer();
        robots_yellow_buffer = RobotMultiCamBuffer();

        updated_cams = std::map<int, bool>();

    }

    inline boost::optional<roboteam_msgs::WorldRobot> botWithId(int id, const std::vector<roboteam_msgs::WorldRobot>& bots) {
    	for (const auto& bot : bots) {
    		if (bot.id == (unsigned) id) {
    			return bot;
    		}
    	}
    	ROS_WARN("FilteredWorld::botWithId: Bot not found: %d", id);
    	return boost::none;
    }

    std::mutex dangerMutex;

    /// Create a message that has the world in it
    roboteam_msgs::World FilteredWorld::as_message() const {

        roboteam_msgs::World returnMsg;

        for (auto& robot : robots_blue_world) {
            returnMsg.them.push_back(robot.second.as_message());
        }

        for (auto& robot : robots_yellow_world) {
            returnMsg.us.push_back(robot.second.as_message());
        }

        returnMsg.ball = ball_world.as_message();

        //TODO : remove danger stuff from here

        std::lock_guard<std::mutex> lock(dangerMutex);
        if (df::DangerFinder::instance().hasCalculated()) {
        	for (int robotID : danger.dangerList) {
                auto bot = botWithId(robotID, returnMsg.them);
        		if (bot) {
        			returnMsg.dangerList.push_back(*bot);
        			returnMsg.dangerScores.push_back(danger.scores.at(robotID));
        			returnMsg.dangerFlags.push_back(danger.flags.at(robotID));
        		}
        	}
        }

        return returnMsg;
    }

    /**
     * To be called when a detectionframe message is received.
     */
    void FilteredWorld::detection_callback(const roboteam_msgs::DetectionFrame msg) {

        
        buffer_detection_frame(msg);

        if (is_calculation_needed()) {

            // Reset the camera update flags.
            for (auto& cam : updated_cams) {
                cam.second = false;
            }

            // double time_grsim = msg.t_capture;
            double time_now = ros::Time::now().toSec();

            merge_frames(time_now);

            fresh = true;
        }

        std::lock_guard<std::mutex> lock(dangerMutex);
        danger = df::DangerFinder::instance().getMostRecentData();

    }

    bool FilteredWorld::isFresh() {
        return fresh;
    }

    void FilteredWorld::setFresh(bool newFresh) {
        fresh = newFresh;
    }

    boost::optional<roboteam_msgs::World> FilteredWorld::consumeMsg() {
        if (isFresh()) {
            setFresh(false);
            return as_message();
        }

        return boost::none;
    }


    /**
     * Adds a received detection frame to the buffers.
     */
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {

        uint cam_id = msg.camera_id;

        // Set this cameras updated flag.
        // If this camera hasn't sent frames before, it is now added to the list of cameras.
        updated_cams[cam_id] = true;

        //TODO: hacked in ignoring cams here for half field play
        /*if (cam_id == 1 || cam_id == 2) {
            ball_buffer.erase(cam_id);
            return;
        }*/

        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            int bot_id = robot.robot_id;

            robots_blue_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            int bot_id = robot.robot_id;

            robots_yellow_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        // Ball
        // Find the ball with the smallest distance from the previous position
        // TODO: Something with speed and extrapolation in case the ball disappears?


        if (msg.balls.size() > 0 /*&& (cam_id == 0 || cam_id == 3)*/) { //TODO: hacked in ignoring cams here for half field play

            Vector2 previousBallPos = ball_buffer[cam_id].pos;

            roboteam_msgs::DetectionBall closestBall = msg.balls[0];
            double closestDist2 = Vector2(closestBall.pos).dist2(previousBallPos);

            for (auto const & ball : msg.balls) {
                Vector2 ballPos(ball.pos);
                
                double dist2 = Vector2(ball.pos).dist2(previousBallPos);
                if (dist2 < closestDist2) {
                    closestBall = ball;
                    closestDist2 = dist2;
                }
            }

            ball_buffer[cam_id] = closestBall; // msg.balls[0];
        } else {
            ball_buffer.erase(cam_id);
        }
    }


    /**
     * Returns true when every camera's frame has updated.
     * When there are no cameras, this function will always return false.
     */
    bool FilteredWorld::is_calculation_needed() const {
        if (updated_cams.empty()) {
            // No cameras? No use doing a frame merge calculation.
            return false;
        }

        for (auto& cam : updated_cams) {
            // there cannot be a third camera without a second camera
            if (!cam.second) {
                return false;
            }
        }
        return true;
    }


    /**
     * Merges the frames from all cameras into the final world state.
     */
    void FilteredWorld::merge_frames(double timestamp) {
        std::string s;
        get_PARAM_OUR_COLOR(s);
        bool isBlueOurTeam = s == "blue";
        merge_robots(robots_blue_buffer, robots_blue_world, old_blue, timestamp, isBlueOurTeam);
        merge_robots(robots_yellow_buffer, robots_yellow_world, old_yellow, timestamp, !isBlueOurTeam);

        // Take the ball from the camera that's closest to the previous position of the ball
        if (ball_buffer.size() > 0) {
            ball_world.set_visible(true);

            Vector2 currentBallPos(ball_world.get_position().x, ball_world.get_position().y);

            roboteam_msgs::DetectionBall closestBall = ball_buffer.begin()->second;
            double closestDist2 = Vector2(closestBall.pos).dist2(currentBallPos);

            for (auto const & ballEntry : ball_buffer) {
                double dist2 = Vector2(ballEntry.second.pos).dist2(currentBallPos);

                if (dist2 < closestDist2) {
                    closestBall = ballEntry.second;
                    closestDist2 = dist2;
                }
            }
            
            ball_world.move_to(closestBall.pos.x, closestBall.pos.y, closestBall.z);
        } else {
            ball_world.set_visible(false);
        }

        // roboteam_msgs::Vector2f speedEstimation = estimateBallSpeed(ball_buffer);
        // ball_world.set_velocity(speedEstimation.x, speedEstimation.y);
        predictor.update(ball_world, timestamp);
        boost::optional<Position> ballVel = predictor.computeBallVelocity();
        if (ballVel) {
            Position vel = *ballVel;
            ball_world.set_velocity(vel.x, vel.y);
        }
        // roboteam_msgs::WorldBall ball = ball_world.as_message();

        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }


    void FilteredWorld::merge_robots(RobotMultiCamBuffer& robots_buffer, std::map<int, rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer, double timestamp, bool our_team) {
        for (auto& robot_buffer : robots_buffer) {
            uint bot_id = robot_buffer.first;

            Robot robot;
            robot.set_id(bot_id);

            float x = 0;
            float y = 0;
            float w = 0;
            Vector2 u(0,0);

            for (auto& buf : robot_buffer.second) {
                x += buf.second.pos.x;
                y += buf.second.pos.y;
                // w += buf.second.orientation;
                // We cant take the arithmic mean of angles here. We have to convert to unit vectors first. (https://en.wikipedia.org/wiki/Mean_of_circular_quantities)
                u = u + Vector2(1,0).rotate(buf.second.orientation);
            }
            x = x / robot_buffer.second.size();
            y = y / robot_buffer.second.size();
            // w = w / robot_buffer.second.size();
            w = u.angle();

            robot.move_to(x, y);
            robot.rotate_to(w);

            predictor.update(robot, our_team, timestamp);
            boost::optional<Position> robotVel = predictor.computeRobotVelocity(bot_id, our_team);
            if (robotVel) {
                Position vel = *robotVel;
                robot.set_vel(vel.x, vel.y, vel.rot);
            }

            // Update the last detection time.
            robot.update_last_detection_time(timestamp);

            robots_output[bot_id] = robot;
            old_buffer[bot_id] = robot;
        }

        // Remove old robots.
        std::map<int, rtt::Robot>::iterator botIter = robots_output.begin();

        while (botIter != robots_output.end()) {
            // Remove robots that are not detected for 0.5 seconds.
            // TODO: Make a ros param for this?
            if (botIter->second.is_detection_old(timestamp, 0.5)) {
                ROS_INFO("Removing bot: %i. Too old.", botIter->second.get_id());
                botIter = robots_output.erase(botIter);
            } else if (botIter->second.is_detection_from_future(timestamp)) {
                ROS_INFO("Removing bot: %i. It's from the future?", botIter->second.get_id());
                botIter = robots_output.erase(botIter);
            } else {
                ++botIter;
            }
        }
    }
}
