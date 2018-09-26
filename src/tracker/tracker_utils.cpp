#include "roboteam_world/tracker/tracker_utils.h"
#include <iostream>

namespace rtt {
    
bool CountingTrackerBase::want_update() {
    // returns true if counter is divisible by update frequency
    return counter++ % update_frequency == 0;
}  

BackgroundTrackerBase::BackgroundTrackerBase(bool allow_skip) :  
                                                  latest_world(),
                                                  should_stop(false),
                                                  has_new(false),
                                                  runner(boost::thread([&]() { dispatcher(); })),
                                                  mutex(),
                                                  allow_skip(allow_skip) {
}

BackgroundTrackerBase::~BackgroundTrackerBase() {
    stop();
}

void BackgroundTrackerBase::update(const World& world) {
    if (should_stop) return;
    if (allow_skip) {
        // Skip update if the previous one is still running
        lock l(mutex, boost::interprocess::try_to_lock);
        if (l.owns()) {
            latest_world = boost::optional<const World&>(world);
            has_new = true;
        }
    } else {
        // Block for previous update to end if it's still running
        while (has_new) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        }
        lock l(mutex);
        latest_world = boost::optional<const World&>(world);
        has_new = true;
    }
}

TrackerResult BackgroundTrackerBase::_calculate_in_background(const TeamRobot& bot) {
    lock l(mutex);
    return calculate_for(bot);
}

std::future<TrackerResult> BackgroundTrackerBase::calculate_in_background(const TeamRobot& bot) {
    return std::async(std::launch::async, &BackgroundTrackerBase::_calculate_in_background, this, bot);
}

void BackgroundTrackerBase::dispatcher() {
    ROS_DEBUG("tracker dispacther");
    while (!should_stop || (BG_TRACKER_WAIT_FOR_FINAL_UPDATE && has_new)) {
        if (has_new && want_update()) {
            lock l(mutex);
            update_impl(*latest_world);
            has_new = false;
        }
    }
}  
    
void BackgroundTrackerBase::start() {
    ROS_DEBUG("tracker start");

    lock l(mutex);
    if (!should_stop) {
        runner.detach();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    } else {
        runner = boost::thread([&]() { dispatcher(); });
        should_stop = false;
        runner.detach();
    }
}    

void BackgroundTrackerBase::stop() {
    ROS_DEBUG("tracker stop");

    lock l(mutex);
    if (!should_stop) {
        should_stop = true;
        runner.join();
    }
}
    
}