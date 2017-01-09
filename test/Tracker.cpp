#include <gtest/gtest.h>
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_world/tracker/opponent_tracker.h"
#include "roboteam_world/predictor.h"
#include "roboteam_world/ros_handler.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "ros/ros.h"
#include <functional>
#include <thread>
#include <vector>
#include <cstdio>

namespace rtt {
    
using namespace roboteam_msgs;


Predictor* pred;
FilteredWorld* world;
RosHandler* handler;

void move_bot(DetectionFrame& frame, unsigned int id, double dx, double dy, double dr, const volatile bool* terminate) {
    ros::Duration duration(.1);
    DetectionRobot& bot = frame.them.at(id);
    while (!*terminate) {
        bot.pos.x += dx;
        bot.pos.y += dy;
        bot.orientation += dr;
        handler->detection_callback(frame);
        duration.sleep();
    }
}

void assert_close(double x, double y, double epsilon = .1) {
    ASSERT_TRUE(std::abs(x - y) < epsilon);
}

TEST(TrackerTests, straight_line_speed) {
    int zero = 0;
    ros::init(zero, nullptr, "world_test");
    pred = new Predictor();
    world = new FilteredWorld(*pred);
    handler = new RosHandler();
    
    handler->init(world);
    DetectionFrame frame;
    frame.them = std::vector<DetectionRobot>();
    frame.them.push_back(DetectionRobot());
    volatile bool stop = false;
    //10 cm / 0.1 s = 1 m/s
    std::thread mover(move_bot, std::ref(frame), 0, .1, 0, 0, &stop);
    mover.detach();
    
    ros::Duration(1).sleep();
    TrackerResult* res = handler->track("Speed", 0);
    ASSERT_TRUE(res->success);
    assert_close(1.0, res->value.pos_val.x);
    stop = true;
    mover.join();
    
    delete handler;
    delete world;
    delete pred;
}
    
}