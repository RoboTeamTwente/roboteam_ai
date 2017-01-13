#include "roboteam_world/tracker/tracker_utils.h"
#include <iostream>

namespace rtt {
    
bool CountingTrackerBase::want_update() {
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
    if (allow_skip) {
        // Skip update if the previous one is still running
        lock l(mutex, boost::interprocess::try_to_lock);
        latest_world = boost::optional<const World&>(world);
        has_new = true;
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

TrackerResult BackgroundTrackerBase::_calculate_in_background(const RobotID& id) {
    lock l(mutex);
    return calculate_for(id);
}

std::future<TrackerResult> BackgroundTrackerBase::calculate_in_background(const RobotID& id) {
    return std::async(std::launch::async, &BackgroundTrackerBase::_calculate_in_background, this, id);
}

void BackgroundTrackerBase::dispatcher() {
    while (!should_stop) {
        if (has_new && want_update()) {
            lock l(mutex);
            if (l.owns()) {
                update_impl(*latest_world);
            } else {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
            has_new = false;
        }
    }
}  
    
void BackgroundTrackerBase::start() {
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
    lock l(mutex);
    if (!should_stop) {
        should_stop = true;
        runner.join();
    }
}
    
}