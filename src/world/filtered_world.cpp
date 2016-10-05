#include "roboteam_world/world/filtered_world.h"


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


    roboteam_msgs::World FilteredWorld::as_message() {
        ROS_INFO("Converting to message");

        roboteam_msgs::World msg;

        for (rtt::Robot& robot : robots_blue_world) {
            msg.robots_blue.push_back(robot.as_message());
        }

        for (rtt::Robot& robot : robots_yellow_world) {
            msg.robots_yellow.push_back(robot.as_message());
        }

        msg.ball = ball_world.as_message();

        ROS_INFO("/Converting to message");

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

            // Reset the camera update flags.
            for (auto& cam : updated_cams) {
                cam.second = false;
            }

            merge_frames();
        }

        ROS_INFO("End callback");
    }


    /**
     * Adds a received detection frame to the buffers.
     */
    void FilteredWorld::buffer_detection_frame(const roboteam_msgs::DetectionFrame msg) {
        ROS_INFO("Buffer frame");

        uint cam_id = msg.camera_id;

        // Set this cameras updated flag.
        // If this camera hasn't sent frames before, it is now added to the list of cameras.
        updated_cams[cam_id] = true;

        ROS_INFO("Blue robots");

        for (const roboteam_msgs::DetectionRobot robot : msg.robots_blue) {
            int bot_id = robot.robot_id;

            robots_blue_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        ROS_INFO("Yellow robots");

        for (const roboteam_msgs::DetectionRobot robot : msg.robots_yellow) {
            int bot_id = robot.robot_id;

            robots_yellow_buffer[bot_id][cam_id] = roboteam_msgs::DetectionRobot(robot);
        }

        // Ball
        if (msg.balls.size() > 0) {
            ball_buffer = msg.balls[0];
        }

        ROS_INFO("----");

    }


    /**
     * Returns true when every camera's frame has updated.
     */
    bool FilteredWorld::is_calculation_needed() {
        ROS_INFO("Is calculation needed?");
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
        ROS_INFO("Merge frames");

        merge_robots(robots_blue_buffer, robots_blue_world, old_blue);
        merge_robots(robots_yellow_buffer, robots_yellow_world, old_yellow);

        ROS_INFO("Move ball");
        ball_world.move_to(ball_buffer.pos.x, ball_buffer.pos.y, ball_buffer.z);


        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();

        ROS_INFO("/Merge frames");
    }


    void FilteredWorld::merge_robots(RobotMultiCamBuffer& robots_buffer, std::vector<rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer) {
        //robots_output->clear();

        ROS_INFO("Merge robots");

        bool skip_velocity = old_buffer.size() == 0;

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
                float dx, dy, dw;

                auto old = old_buffer[bot_id].as_message();
                dx = x - old.pos.x;
                dy = y - old.pos.y;
                dw = w - old.w;

                robot.set_vel(dx, dy, dw);
            }
            robot.move_to(x, y);
            robot.rotate_to(w);


            robots_output.at(bot_id) = robot;
            old_buffer[bot_id] = robot;
        }

        ROS_INFO("/Merge robots");
    }

}
