#pragma once

#include <map>
#include <boost/optional.hpp>
#include <gtest/gtest_prod.h>
#include "ros/ros.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"

#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include "roboteam_world/predictor.h"

#include "roboteam_world/world/world_base.h"

// TODO: Make sure the us/them nomenclature also propagates to the buffers
// TODO: Make sure the us/them stuff is decided by a parameters settable somewhere

namespace rtt {

    class FilteredWorld : public WorldBase {

    private:
        //TODO: NOdehandle is not used right now?
        ros::NodeHandle nh;
        /**
         * These buffers store for every camera the robots and balls.
         * Accessing goes like this:
         * `robots_blue_buffer[robot_id][camera_id]`
         */
        typedef std::map<int, std::map<int, roboteam_msgs::DetectionRobot>> RobotMultiCamBuffer;
        RobotMultiCamBuffer robots_blue_buffer;
        RobotMultiCamBuffer robots_yellow_buffer;

        std::map<int, roboteam_msgs::DetectionBall> ball_buffer;

        std::map<int, rtt::Robot> old_blue, old_yellow;

        // Keeps track which cameras have sent a frame since last world calculation.
        // Also keeps track of which cameras are on-line and sending frames.
        std::map<int, bool> world_cams;

        /**
         * Final world state being converted to a message when
         * `as_message()` is called.
         */
        std::map<int, rtt::Robot> robots_yellow_world;
        std::map<int, rtt::Robot> robots_blue_world;
        rtt::Ball ball_world;

        Predictor predictor;

        bool fresh;

    public:
        explicit FilteredWorld(Predictor predictor);

        /**
        * Resets the world.
         *
        */
        void reset() override;

        /**
         * Converts this world into a ros message.
         */
        roboteam_msgs::World as_message() const override;

        /**
         * To be called when a detectionframe message is received.
         */
        void detection_callback(roboteam_msgs::DetectionFrame msg) override;

        //TODO: Make isFresh() and setFresh() private? They are not used publicly as far as I can tell.
        /**
         * If a new frame is available will return true
         */
        bool isFresh();

        /**
         * Can set fresh to false if the new frame is consumed.
         */
        void setFresh(bool newFresh);

        /**
         * Calls as_message and sets fresh to false. If isFresh() is false
         * returns boost::none.
         */
        boost::optional<roboteam_msgs::World> consumeMsg();

    private:

        // Allows for testing of private methods
        FRIEND_TEST(WorldTests, filtered);

        /**
         * Puts a received detection frame in the associated camera's buffer.
         */
        void buffer_detection_frame(roboteam_msgs::DetectionFrame msg);

        /**
         * Returns true when every camera's frame has updated.
         */
        bool is_calculation_needed() const;

        /**
         * Merges the frames from all cameras into the final world state.
         */
        void merge_frames(double timestamp);

        void merge_robots(RobotMultiCamBuffer& robots_buffer, std::map<int, rtt::Robot>& robots_output, std::map<int, rtt::Robot>& old_buffer, double timestamp, bool our_team);

    };

}
