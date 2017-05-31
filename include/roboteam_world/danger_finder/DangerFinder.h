#pragma once

#include <mutex>
#include <thread>
#include <vector>
#include <map>
#include "DangerModule.h"
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/DangerFinder.h"
#include "roboteam_world/world/world_base.h"
#include "roboteam_world/danger_finder/DangerData.h"
#include "ros/ros.h"

namespace rtt {

namespace df {

/**
 * \class DangerFinder
 * \brief Utility which continously (in a background thread) monitors the world state and
 * keeps track of which opponents pose the greatest threat.
 */
class DangerFinder {
public:
	static WorldBase* world;
	// It's a singleton; don't copy it.
	DangerFinder(const DangerFinder&) = delete;
	void operator=(const DangerFinder&) = delete;
	virtual ~DangerFinder(){}

	/**
	 * \function getMostRecentData
	 * \brief Gets the most recent results of the DangerFinder thread.
	 * If the background thread has not been started it, this starts it.
	 */
	virtual DangerData getMostRecentData();

	/**
	 * \function calculateDataNow
	 * \brief Performs an immediate update of the DangerData.
	 * The returned data is also stored as the 'most recent'.
	 */
	virtual DangerData calculateDataNow();

	/**
	 * \function start
	 * \brief Starts the background worker thread.
	 * \param iterationsPerSecond The amount of iterations the background thread should try to run per second.
	 */
	virtual void start(int iterationsPerSecond = 20);

	/**
	 * \function stop
	 * \brief Stops the background worker thread.
	 */
	virtual void stop();

	/**
	 * \function isRunning
	 * \brief Checks whether the background thread is running.
	 */
	virtual bool isRunning() const;

	/**
	 * \function instance
	 * \brief Gets the singleton instance.
	 */
	static DangerFinder& instance();

	/**
	 * \function instance
	 * \brief Starts the background thread if it hasn't been started yet.
	 * Does nothing if the thread has started already.
	 */
	static void ensureRunning(int iterationsPerSecond = 20);
private:
	static std::vector<df::DangerModule*> modules();
	std::thread runner;
	std::mutex  mutex;
	volatile bool stopping;
	volatile bool running;
	bool ranOnce;
	DangerData mostRecentData;

	void calculate();
	void loop(unsigned delayMillis);
protected:
	DangerFinder();
};

}

}
