#include "roboteam_world/world/filtered_world.h"
#include <vector>
#include "roboteam_utils/Vector2.h"

namespace rtt {

    FilteredWorld::FilteredWorld() {
        reset();
    }

    void FilteredWorld::reset() {

        // Reset all vectors.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();

        robots_blue_world.clear();
        robots_yellow_world.clear();
        ball_world = rtt::Ball();

        // Initialize the input buffers.
        robots_blue_buffer = RobotMultiCamBuffer();
        robots_yellow_buffer = RobotMultiCamBuffer();

        updated_cams = std::map<int, bool>();
    }


    roboteam_msgs::World FilteredWorld::as_message() const {

        roboteam_msgs::World msg;

        for (const rtt::Robot& robot : robots_blue_world) {
            msg.them.push_back(robot.as_message());
        }

        for (const rtt::Robot& robot : robots_yellow_world) {
            msg.us.push_back(robot.as_message());
        }

        msg.ball = ball_world.as_message();

        return msg;
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

            merge_frames();
        }
    }


    /**
     * Adds a received detection frame to the buffers.
     */
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {

        uint cam_id = msg.camera_id;

        // Set this cameras updated flag.
        // If this camera hasn't sent frames before, it is now added to the list of cameras.
        updated_cams[cam_id] = true;

        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            int bot_id = robot.robot_id;

            robots_blue_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            int bot_id = robot.robot_id;

            robots_yellow_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        // Ball
        if (msg.balls.size() > 0) {
            ball_buffer = msg.balls[0];
        }

    }


    /**
     * Returns true when every camera's frame has updated.
     * When there are no cameras, this function will always return false.
     */
    bool FilteredWorld::is_calculation_needed() const {
        if (updated_cams.size() == 0) {
            // No cameras? No use doing a frame merge calculation.
            return false;
        }

        for (auto& cam : updated_cams) {
            if (!cam.second) {
                return false;
            }
        }
        return true;
    }


    /**
     * Merges the frames from all cameras into the final world state.
     */
    void FilteredWorld::merge_frames() {

        merge_robots(robots_blue_buffer, robots_blue_world, old_blue);
        merge_robots(robots_yellow_buffer, robots_yellow_world, old_yellow);

        ball_world.move_to(ball_buffer.pos.x, ball_buffer.pos.y, ball_buffer.z);
        roboteam_msgs::Vector2f speedEstimation = estimateBallSpeed(ball_buffer);
        // ROS_INFO_STREAM(speedEstimation.x << " " << speedEstimation.y);
        ball_world.set_velocity(speedEstimation.x, speedEstimation.y);

        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }


    void FilteredWorld::merge_robots(RobotMultiCamBuffer& robots_buffer, std::vector<rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer) {
        //robots_output->clear();

        bool skip_velocity = old_buffer.size() == 0;
        // ROS_INFO_STREAM("new");
        for (auto& robot_buffer : robots_buffer) {
            uint bot_id = robot_buffer.first;

            Robot robot;

            robot.set_id(bot_id);

            // Resize the vector so that this id fits in.
            // The vector should probably be changed to a map.
            if (robots_output.size() <= bot_id) {
                robots_output.resize(bot_id + 1);
            }

            float x = 0;
            float y = 0;
            float w = 0;

            for (auto& buf : robot_buffer.second) {
                x += buf.second.pos.x;
                y += buf.second.pos.y;
                w += buf.second.orientation;
            }
            x = x / robot_buffer.second.size();
            y = y / robot_buffer.second.size();
            w = w / robot_buffer.second.size();

            if (!skip_velocity) {	
										
				roboteam_utils::Vector2 robotPos;
                robotPos.x = x;
                robotPos.y = y;
                roboteam_utils::Vector2 estimatedSpeed = estimateRobotSpeed(bot_id, robotPos);

                robot.set_vel(estimatedSpeed.x, estimatedSpeed.y, 0);
                
            }
            robot.move_to(x, y);
            robot.rotate_to(w);


            robots_output.at(bot_id) = robot;
            old_buffer[bot_id] = robot;
        }
    }

    roboteam_utils::Vector2 FilteredWorld::estimateRobotSpeed(uint bot_id, roboteam_utils::Vector2 robotPos) {

        std::vector<roboteam_utils::Vector2>::iterator it;
        it = robots_pos_history[bot_id].begin();
        robots_pos_history[bot_id].insert(it, robotPos);

        int smoothingNumber = 5; // positive integer
        double timeStep = 1.0/60.0; // seconds
		// could be improved by actually measuring the timestep and saving it in the map
		
		
        if (robots_pos_history[bot_id].size() > (size_t)smoothingNumber) {
            robots_pos_history[bot_id].erase(robots_pos_history[bot_id].end());
        }


        roboteam_utils::Vector2 posDiff;

        for (size_t i = 0; i < (robots_pos_history[bot_id].size()-1); i++) {
            roboteam_msgs::Vector2f posOld = robots_pos_history[bot_id].at(i+1);
            roboteam_msgs::Vector2f pos = robots_pos_history[bot_id].at(i);
            posDiff.x += pos.x - posOld.x;
            posDiff.y += pos.y - posOld.y;
        }

        roboteam_utils::Vector2 speedEstimation;
        speedEstimation.x = posDiff.x / smoothingNumber / timeStep;
        speedEstimation.y = posDiff.y / smoothingNumber / timeStep;


        return speedEstimation;
    }

    roboteam_msgs::Vector2f FilteredWorld::estimateBallSpeed(roboteam_msgs::DetectionBall ball_buffer) {
        std::vector<roboteam_msgs::DetectionBall>::iterator it;
        it = old_ball_positions.begin();
        old_ball_positions.insert(it, ball_buffer);

        int smoothingNumber = 5; // positive integer
        double timeStep = 1.0/60.0; // seconds

        if (old_ball_positions.size() > (size_t)smoothingNumber) {
            old_ball_positions.erase(old_ball_positions.end());
        }

        roboteam_msgs::Vector2f posDiff;
        for (size_t i = 0; i < (old_ball_positions.size()-1); i++) {
            roboteam_msgs::Vector2f posOld = old_ball_positions[i+1].pos;
            roboteam_msgs::Vector2f pos = old_ball_positions[i].pos;
            posDiff.x += pos.x - posOld.x;
            posDiff.y += pos.y - posOld.y;
        }

        roboteam_msgs::Vector2f speedEstimation;
        speedEstimation.x = posDiff.x / smoothingNumber / timeStep;
        speedEstimation.y = posDiff.y / smoothingNumber / timeStep;

        return speedEstimation;
    }

}
