#include <gtest/gtest.h>
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_world/tracker/tracker.h"
#include "roboteam_world/tracker/tracker_utils.h"
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
#include <iostream>

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

/*
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
    assert_close(1.0, boost::get<Position>(res->value).x);
    stop = true;
    mover.join();
    
    delete handler;
    delete world;
    delete pred;
}
*/
    
class CounterTest : public CountingTrackerBase {
    public:
    CounterTest() : CountingTrackerBase(5), count(0) {}
    int count;
    void update(const World& world) override { count++; }
    virtual TrackerResult calculate_for(const TeamRobot& bot) const override {
        return TrackerResult();
    }
    const std::string name() const { return "CounterTest"; }
}; 

class BackgroundTest : public BackgroundTrackerBase {
public:
    BackgroundTest(bool allow_skip) : BackgroundTrackerBase(allow_skip), count(0) {}
    volatile int count;
    void start() { BackgroundTrackerBase::start(); }
    void stop() { BackgroundTrackerBase::stop(); }
    void update_impl(const World& world) override { 
        boost::this_thread::sleep(boost::posix_time::milliseconds(50)); // expensive calculation!!
        count++; 
    }
    virtual TrackerResult calculate_for(const TeamRobot& bot) const override {
        boost::this_thread::sleep(boost::posix_time::milliseconds(200)); // expensive calculation!!
        return TrackerResult();
    }
    const std::string name() const { return "BackgroundTest"; }    
};

TEST(TrackerTests, tracker_utils_test) {
    World dummy_world;
    
    CounterTest counter;
    for (int i = 0; i < 50; i++) {
        // want_update is normally called from OpponentTracker::update
        if (counter.want_update()) {
            counter.update(dummy_world);
        }
    }
    // 1 in 5 updates should be processed
    ASSERT_EQ(10, counter.count);
    
    BackgroundTest background_skip(true);  // Non-blocking, will skip updates if still busy.
    BackgroundTest background_wait(false); // Blocking, will process all updates, blocking if required.
    
    // 
    background_skip.start();
    for (int i = 0; i < 50; i++) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        background_skip.update(dummy_world);
    }
    background_skip.stop();
    
    // Processing takes 10 times as long as updating, so 1 in 10 updates will be processed.
    // 50 updates, so on average 5 will be processed.
    ASSERT_LE(4, background_skip.count);
    ASSERT_GE(6, background_skip.count);
    
    boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();
    background_wait.start();
    
    // Do not wait between updates; they should block
    for (int i = 0; i < 10; i++) {
        background_wait.update(dummy_world);
    }
    
    background_wait.stop();
    boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration duration = end - begin;
    
    // Blocking updates, so the count should be precise
    //ASSERT_EQ(10, background_wait.count);
    //ASSERT_GE(9, background_wait.count); // Temp - last update sometimes fails. Non-critical.
    ASSERT_LE(10, background_wait.count);
    
    // Processing will require 500ms, but allow a little more time for other operations
    ASSERT_LE(500, duration.total_milliseconds());
    ASSERT_GE(550, duration.total_milliseconds());
    
    TeamRobot bot = {0, true};
    std::future<TrackerResult> future = background_wait.calculate_in_background(bot);
    begin = boost::posix_time::microsec_clock::local_time();
    future.wait();
    end = boost::posix_time::microsec_clock::local_time();
    duration = end - begin;
    ASSERT_LE(200, duration.total_milliseconds());
    ASSERT_GE(210, duration.total_milliseconds());
    
}
    
}