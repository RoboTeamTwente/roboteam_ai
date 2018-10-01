#pragma once

#include <mutex>
#include <thread>
#include <vector>
#include <map>
#include "modules/DangerModule.h"
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_world/world/world_base.h"
#include "DangerData.h"
#include "ros/ros.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

/**
 * \class DangerFinder
 * \brief Utility which continuously (in a background thread) monitors the world state and
 * keeps track of which opponents pose the greatest threat.
 */
class DangerFinder {
 public:
  static roboteam_msgs::World * worldMsg;

  // It's a singleton; don't copy it.
  DangerFinder(const DangerFinder &) = delete;
  void operator=(const DangerFinder &) = delete;

  /**
   * \function getMostRecentData
   * \brief Gets the most recent results of the DangerFinder thread.
   * If the background thread has not been started it, this starts it.
   */
  DangerData getMostRecentData();

  /**
   * \function calculateDataNow
   * \brief Performs an immediate update of the DangerData.
   * The returned data is also stored as the 'most recent'.
   */
  DangerData calculateDataNow();

  /**
   * \function start
   * \brief Starts the background worker thread.
   * \param iterationsPerSecond The amount of iterations the background thread should try to run per second.
   */
  void start(int iterationsPerSecond = 20);

  /**
   * \function stop
   * \brief Stops the background worker thread.
   */
  void stop();

  /**
   * \function isRunning
   * \brief Checks whether the background thread is running.
   */
  bool isRunning() const;

  bool hasCalculated() const;

  /**
   * \function instance
   * \brief Gets the singleton instance.
   */

    static DangerFinder &instance();
    /**
   * \function instance
   * \brief Starts the background thread if it hasn't been started yet.
   * Does nothing if the thread has started already.
   */
  static void ensureRunning(int iterationsPerSecond = 20);
 private:
  void loadModules();
  std::thread runner;
  std::mutex mutex;
  volatile bool stopping;
  volatile bool running;
  bool ranOnce;
  DangerData mostRecentData;
  std::vector<DangerModule *> modules;
  void calculate();
  void drawDanger(DangerData);
  void loop(unsigned delayMillis);
  DangerFinder();
};

} // dangerfinder
} // ai
} // rtt