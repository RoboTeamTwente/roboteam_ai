#include "DangerFinder.h"
#include "ros/ros.h"
#include "modules/DistanceModule.h"
#include "modules/OrientationModule.h"
#include "modules/CanShootModule.h"
#include "modules/FreeModule.h"
#include "modules/WillReceiveBallModule.h"
#include "modules/HasBallModule.h"
#include <boost/optional.hpp>
#include <sstream>

namespace rtt {
namespace ai {
namespace dangerfinder {

roboteam_msgs::World * DangerFinder::worldMsg = nullptr;

/*
 * \class DangerFinder
 * \brief Utility which continuously (in a background thread) monitors the world state and
 * keeps track of which opponents pose the greatest threat.
 */
DangerFinder::DangerFinder() : stopping(false), running(false), ranOnce(false) {

  CanShootModule canShootModule(999);
  modules.push_back(&canShootModule);

  DistanceModule distanceModule(-1.7/9.0); // factor / max width
  modules.push_back(&distanceModule);

  FreeModule freeModule(0.0);
  modules.push_back(&freeModule);

  HasBallModule hasBallModule(100);
  modules.push_back(&hasBallModule);

  OrientationModule orientationModule(1.7, 3.0, 1);
  modules.push_back(&orientationModule);

  WillReceiveBallModule willReceiveBallModule;
  modules.push_back(&willReceiveBallModule);
}

DangerFinder &DangerFinder::instance() {
  static DangerFinder local_df;
  return local_df;
}

void DangerFinder::ensureRunning(int itsPerSecond) {
  if (!instance().running) {
    instance().start(itsPerSecond);
  }
}

void DangerFinder::start(int iterationsPerSecond) {
  loadModules();
  ROS_INFO_STREAM_NAMED("DangerFinder", "Starting at " << iterationsPerSecond << " iterations per second");
  unsigned delay = (unsigned) (1000/iterationsPerSecond);
  runner = std::thread(&DangerFinder::loop, this, delay);
  runner.detach();
  running = true;
}

void DangerFinder::loop(unsigned delayMillis) {
  std::chrono::milliseconds delay(delayMillis);

  while (!stopping) {
    calculate();
    std::this_thread::sleep_for(delay);
  }
}

void DangerFinder::calculate() {
  DangerData data;
  for (const auto &bot : worldMsg->them) {
    PartialResult pr;
    for (auto &module : modules) {
      auto t = module->calculate(bot);
      pr += t;
    }
    data.flags[bot.id] = pr.flags;
    data.scores[bot.id] = pr.score;
    data.dangerList.push_back(bot.id);
  }
  std::sort(data.dangerList.begin(), data.dangerList.end(), [data](const int &a, const int &b) {
    if (data.scores.find(a)==data.scores.end() || data.scores.find(b)==data.scores.end()) {
      ROS_WARN("DangerFinder::calculate: An element of dangerList was not a key in data.scores; sorting failed.");
      return false;
    }
    return data.scores.at(a) > data.scores.at(b);
  });

  std::lock_guard<std::mutex> lock(mutex);
  mostRecentData = data;
  ranOnce = true;
}

roboteam_msgs::WorldRobot findBot(int id, const roboteam_msgs::World &world) {
  for (const auto &bot : world.them) {
    if (bot.id==(unsigned) id) {
      return bot;
    }
  }
  throw new std::invalid_argument("DangerFinder.cpp:findBot - bot not found");
}

bool DangerFinder::isRunning() const {
  return running;
}

void DangerFinder::stop() {
  stopping = true;
  runner.join();
  running = false;
}

/**
 * \function getMostRecentData
 * \brief Gets the most recent results of the DangerFinder thread.
 * If the background thread has not been started it, this starts it.
 */
DangerData DangerFinder::getMostRecentData() {
  ensureRunning();
  if (!ranOnce) {
    calculate();
  }
  DangerData t;
  std::lock_guard<std::mutex> lock(mutex);
  t = mostRecentData;
  return t;
}

DangerData DangerFinder::calculateDataNow() {
  calculate();
  return getMostRecentData();
}

bool DangerFinder::hasCalculated() {
  std::lock_guard<std::mutex> lock(mutex);
  return ranOnce;
}

} // dangerfinder
} // ai
} // rtt