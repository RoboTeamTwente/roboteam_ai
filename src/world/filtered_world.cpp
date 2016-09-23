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

        buffer_detection_frame(msg);

        if (is_calculation_needed()) {
            ROS_INFO("Calculation needed!");
            updated_cams = std::vector<bool>(config.num_cams(), false);
        }

        merge_frames();

    }


    /**
     * Adds a received detection frame to the buffers.
     */
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {
        int cam_id = msg.camera_id;

        if (cam_id < config.num_cams()) {

            // Set this cameras updated flag.
            updated_cams[cam_id] = true;

            for (auto& robot : msg.robots_blue) {
                int bot_id = robot.robot_id;

                robots_blue_buffer[bot_id].resize(config.num_cams());
                robots_blue_buffer[bot_id][bot_id] = robot;
            }

            for (auto& robot : msg.robots_yellow) {
                int bot_id = robot.robot_id;

                robots_yellow_buffer[bot_id].resize(config.num_cams());
                robots_yellow_buffer[bot_id][bot_id] = robot;
            }

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
        // Clear the final vectors.
        robots_blue_world.clear();
        robots_yellow_world.clear();
        ball_world = rtt::Ball();


        // ---- Merge here ----


        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }

}
