#pragma once

#include "roboteam_world/tracker/opponent_tracker.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <future>
#include <boost/optional.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/lock_options.hpp>

namespace rtt {
    
typedef boost::interprocess::scoped_lock<boost::mutex> lock;    
 
/**
 * @class CountingTrackerBase
 * @author Dennis
 * @date 12/01/17
 * @file tracker_utils.h
 * @brief Helper for TrackerModules which will trigger calculations every X updates,
 * where X is the constructor parameter.
 */   
class CountingTrackerBase : virtual public TrackerModule {
public:    
    virtual ~CountingTrackerBase() {}
    
    /**
     * @param frequency update() will be called once in every 'frequency' updates.
     */
    CountingTrackerBase(unsigned int frequency) : update_frequency(frequency) {}
    
    /**
     * @brief Enforces the '1 in frequency' rule.
     * @return True every frequency'th time.
     */
    bool want_update() final override;
    
protected:
    const unsigned int update_frequency;
private:
    int counter;
    
};

/**
 * @class BackgroundTrackerBase
 * @author Dennis
 * @date 12/01/17
 * @file tracker_utils.h
 * @brief Helper for expensive TrackerModules which want to execute in a background thread.
 * These may be configured to skip updates if they are still busy, or just to not block
 * until a new update comes in while the previous one is still being processed. Also
 * provides the possibility for returning TrackerResults as futures.
 */
class BackgroundTrackerBase : virtual public TrackerModule {
public:    
    /**
     * @param allow_skip
     *    If true:
     *      The module will process updates in a background thread. update() will never block.
     *      If update() is called while another update is still being processed, that update
     *      will be ignored.
     *    If false:
     *      Processing still takes place in the background, but every update must be processed.
     *      Calling update() while the runner thread is idle will not block, but if the runner thread
     *      is currently processing another update, the update() method will block until that previous
     *      calculation has completed.
     */
    BackgroundTrackerBase(bool allow_skip = true);
    virtual ~BackgroundTrackerBase();
    virtual void update(const World& world) final override;
    
    /**
     * @brief Get a future to the latest tracker result for a robot. This effectively
     * calls calculate_for(id) in the background.
     * @param id The robot to track.
     * @return A future which will at some point contain the result of calculate_for(id).
     */
    std::future<TrackerResult> calculate_in_background(const RobotID& id);
protected:    
    /**
     * @brief Performs the actual update of the module's internal state.
     * @param latest_world The most recent world state.
     */
    virtual void update_impl(const World& latest_world) = 0;
    
    /**
     * @brief Starts the background thread, or restarts it. If the background thread is
     * already running, this method has no effect.
     */
    void start();
    
    /**
     * @brief Stops the background thread. Any calculation currently being processed will
     * be allowed to finish. If the thread is not running, this method has no effect.
     */
    void stop();
private:
    boost::optional<const World&> latest_world;
    volatile bool should_stop;
    volatile bool has_new;
    boost::thread runner;
    boost::mutex mutex;
    const bool allow_skip;

    void dispatcher();
    TrackerResult _calculate_in_background(const RobotID& id);
};
    
}