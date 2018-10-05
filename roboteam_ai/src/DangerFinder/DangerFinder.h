#pragma once

#include <mutex>
#include <thread>
#include <vector>
#include <map>
#include "modules/DangerModule.h"
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "DangerData.h"
#include "ros/ros.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class DangerFinder {
 public:
  static roboteam_msgs::World *worldMsg;
  DangerFinder(const DangerFinder &) = delete;  // It's a singleton; don't copy it.
  void operator=(const DangerFinder &) = delete;
  DangerData getMostRecentData();
  DangerData calculateDataNow();
  void start(int iterationsPerSecond = 20);
  void stop();
  bool isRunning() const;
  bool hasCalculated();
  static DangerFinder &instance();

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
  void loop(unsigned delayMillis);
  DangerFinder();
};
} // dangerfinder
} // ai
} // rtt