#include "DangerFinder.h"
#include "ros/ros.h"
#include "modules/DistanceModule.h"
#include "modules/OrientationModule.h"
#include "roboteam_utils/Draw.h"
#include <boost/optional.hpp>
#include <sstream>

namespace rtt {
namespace ai {
namespace dangerfinder {

WorldBase *DangerFinder::world = nullptr;

boost::optional<roboteam_msgs::WorldRobot> getWorldBot(int id, bool ourTeam) {
  auto world = LastWorld::get();
  std::vector<roboteam_msgs::WorldRobot> vec = ourTeam ? world.us : world.them;
  for (const auto &bot : vec) {
    if (bot.id==(unsigned) id) return bot;
  }
  return boost::none;
}

std::vector<DangerModule *> DangerFinder::modules() {
  static std::vector<DangerModule *> *vec = nullptr;
  if (vec==nullptr) {
    ROS_INFO("Building modules...");
    vec = new std::vector<DangerModule *>;
    auto config = DangerModule::cfg();
    for (std::string moduleName : config.getActiveModules()) {
      ROS_INFO_STREAM_NAMED("DangerFinder", "Module activated: " << moduleName);
      boost::optional<DangerModule *> optMod = DangerModule::buildModule(moduleName);
      if (!optMod) {
        ROS_WARN_STREAM_NAMED("DangerFinder", "Module with name '" << moduleName
                                                                   << "' listed in the config file, but not registered");
      } else {
        vec->push_back(*optMod);
      }
    }
  }
  return *vec;
};

DangerFinder::DangerFinder() : stopping(false), running(false), ranOnce(false) {}

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
  const auto worldMsg = world->as_message();
  for (const auto &bot : worldMsg.them) {
//		ROS_DEBUG_NAMED("DangerFinder", "Calculating danger for bot %d", bot.id);
    PartialResult pr;
    for (auto &module : modules()) {
      auto t = module->calculate(bot);
//			ROS_DEBUG_NAMED("DangerFinder", "Module %s: Score=%f Flags=0x%02X", module->getName().c_str(), t.score, t.flags);
      pr += t;
    }
//		ROS_DEBUG_NAMED("DangerFinder", "Results: Score=%f, Flags=0x%02X\n", pr.score, pr.flags);
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

//	ROS_DEBUG_NAMED("DangerFinder", "Danger list: [ ");
//	for (int i : data.dangerList) {
//		ROS_DEBUG_NAMED("DangerFinder", "%d ", i);
//	}
//	ROS_DEBUG_NAMED("DangerFinder", "]\n");

//	drawDanger(data);
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

void DangerFinder::drawDanger(DangerData data) {
  static Draw draw;
  double minScore = 9999999, maxScore = -9999999;
  for (const auto &pair : data.scores) {
    double score = pair.second;
    if (score > maxScore) maxScore = score;
    if (score < minScore) minScore = score;
  }
  const auto worldMsg = world->as_message();

  try {
    for (int id : data.dangerList) {
      const auto bot = findBot(id, worldMsg);
      double myScore = data.scores.at(id);
      double relScore = (myScore - minScore)/(maxScore - minScore);
      int redness = 255*relScore;
      int greenness = 50*relScore;
      int blueness = 255 - (255*relScore);
      std::ostringstream ss;
      ss << "Bot" << id << "DangerLine";
      draw.setColor(redness, greenness, blueness);
      draw.drawLine(ss.str(), {bot.pos.x - .09, bot.pos.y + .11}, {relScore*.18, 0});
    }
  } catch (std::invalid_argument const &e) {
    ROS_ERROR(
        "Error in DangerFinder at line %d: %s. Result: drawing of danger bars in RQT will be incorrect. This is a bug!",
        __LINE__,
        e.what()
    );
  }
}

bool DangerFinder::isRunning() const {
  return running;
}

void DangerFinder::stop() {
  stopping = true;
  runner.join();
  running = false;
}

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

bool DangerFinder::hasCalculated() const {
  return ranOnce;
}

} // dangerfinder
} // ai
} // rtt