#include "filtered_world.h"


namespace rtt {

    FilteredWorld::FilteredWorld() {

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

        updated_cams = std::vector<bool>(config.num_cams(), false);
    }


    roboteam_msgs::World FilteredWorld::as_message() {
        roboteam_msgs::World msg;

        for (std::vector<rtt::Robot>::iterator it = robots_blue_world.begin(); it != robots_blue_world.end(); it++) {
            msg.robots_blue.push_back(it->as_message());
        }

        for (std::vector<rtt::Robot>::iterator it = robots_yellow_world.begin(); it != robots_yellow_world.end(); it++) {
            msg.robots_yellow.push_back(it->as_message());
        }

        msg.ball = ball_world.as_message();

        return msg;
    }


    /**
     * To be called when a detectionframe message is received.
     */
    void FilteredWorld::detection_callback(const roboteam_msgs::DetectionFrame msg) {

        ROS_INFO("Message received.");

        buffer_detection_frame(msg);

        if (is_calculation_needed()) {
            ROS_INFO("Calculation needed!");
            updated_cams = std::vector<bool>(config.num_cams(), false);

            merge_frames();
        }
    }


    /**
     * Adds a received detection frame to the buffers.
     */
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {
        ROS_INFO("Buffer frame");

        int cam_id = msg.camera_id;

        if (cam_id < config.num_cams()) {

            // Set this cameras updated flag.
            updated_cams[cam_id] = true;

            ROS_INFO("Blue robots");

            for (auto& robot : msg.robots_blue) {
                int bot_id = robot.robot_id;

                robots_blue_buffer[bot_id].resize(config.num_cams());
                robots_blue_buffer[bot_id][cam_id] = robot;
            }

            ROS_INFO("Yellow robots");

            for (auto& robot : msg.robots_yellow) {
                int bot_id = robot.robot_id;

                robots_yellow_buffer[bot_id].resize(config.num_cams());
                robots_yellow_buffer[bot_id][cam_id] = robot;
            }

            // Ball
            if (msg.balls.size() > 0) {
                ball_buffer = msg.balls[0];
            }

            ROS_INFO("----");

        }
    }


    /**
     * Returns true when every camera's frame has updated.
     */
    bool FilteredWorld::is_calculation_needed() {
        for (int i = 0; i < updated_cams.size(); ++i) {
            if (!updated_cams[i]) {
                return false;
            }
        }
        return true;
    }


    /**
     * Merges the frames from all cameras into the final world state.
     */
    void FilteredWorld::merge_frames() {
        merge_robots(&robots_blue_buffer, &robots_blue_world);
        merge_robots(&robots_yellow_buffer, &robots_yellow_world);

        ball_world.move_to(ball_buffer.x, ball_buffer.y, ball_buffer.z);

        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }


    void FilteredWorld::merge_robots(RobotMultiCamBuffer* robots_buffer, std::vector<rtt::Robot>* robots_output) {
        //robots_output->clear();

        for (auto& robot_buffer : *robots_buffer) {
            rtt::Robot robot = rtt::Robot();

            int bot_id = robot_buffer.first;

            robot.set_id(bot_id);

            // Resize the vector so that this id fits in.
            // The vector should probably be changed to a map.
            if (robots_output->size() <= bot_id) {
                robots_output->resize(bot_id + 1);
            }

            float x = 0;
            float y = 0;
            float w = 0;
            for (auto& buf : robot_buffer.second) {
                x += buf.x;
                y += buf.y;
                w += buf.orientation;
            }
            x = x / robot_buffer.second.size();
            y = y / robot_buffer.second.size();
            w = w / robot_buffer.second.size();

            robot.move_to(x, y);
            robot.rotate_to(w);

            robots_output->at(bot_id) = robot;
        }
    }

}
