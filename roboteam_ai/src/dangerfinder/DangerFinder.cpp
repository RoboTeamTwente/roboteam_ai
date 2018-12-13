#include "DangerFinder.h"
#include <bitset>

namespace rtt {
namespace ai {
namespace dangerfinder {

// Utility which continuously (in a background thread) monitors the world state and
// keeps track of which opponents pose the greatest threat.
DangerFinder::DangerFinder() : stopping(false), running(false), ranOnce(false) { }

// Gets the singleton instance.
DangerFinder &DangerFinder::instance() {
    static DangerFinder local_df;
    return local_df;
}

// Starts the background thread if it hasn't been started yet. Does nothing if the thread has started already.
// \param iterationsPerSecond The amount of iterations the background thread should try to run per second.
void DangerFinder::ensureRunning(int iterationsPerSecond) {
    if (! instance().running) {
        instance().start(iterationsPerSecond);
    }
}

// Starts the background worker thread.
// \param iterationsPerSecond The amount of iterations the background thread should try to run per second.
void DangerFinder::start(int iterationsPerSecond) {
    loadModules();
    ROS_INFO_STREAM_NAMED("dangerfinder",
            "Starting at " << iterationsPerSecond << " iterations per second");
    auto delay = (unsigned) (1000/iterationsPerSecond);
    runner = std::thread(&DangerFinder::loop, this, delay);
    running = true;
}

// Run the calculation loop
// \params delayMillis: the amount of milliseconds to delay before calculating another thread
// TODO this is slower than desired FPS if calculations take long
void DangerFinder::loop(unsigned delayMillis) {
    std::chrono::milliseconds delay(delayMillis);

    while (! stopping) {
        calculate();
        std::this_thread::sleep_for(delay);
    }
}

// calculate all danger scores and store them in mostRecentData
void DangerFinder::calculate() {
    DangerData data;
    for (const auto &bot : rtt::ai::World::get_world().them) {
        PartialResult pr;
        for (auto &dangerModule : dangerModules) {
            PartialResult result = dangerModule->calculate(bot);
            pr += result;
        }
        data.flags[bot.id] = pr.flags;
        data.scores[bot.id] = pr.score;
        data.dangerList.push_back(bot.id);
    }

    std::sort(data.dangerList.begin(), data.dangerList.end(), [data](const int &a, const int &b) {
      if (data.scores.find(a) == data.scores.end() || data.scores.find(b) == data.scores.end()) {
          ROS_WARN(
                  "dangerfinder::calculate: An element of dangerList was not a key in data.scores; sorting failed.");
          return false;
      }
      return data.scores.at(a) > data.scores.at(b);
    });

    std::lock_guard<std::mutex> lock(mutex);
    mostRecentData = data;
    ranOnce = true;
}

// Stops the background worker thread.
void DangerFinder::stop() {

    if (running) {
        ROS_INFO_STREAM_NAMED("dangerfinder", "Stopping dangerfinder");
        dangerModules.clear();
        stopping = true;
        runner.join();
        running = false;
    }
    else {
        ROS_INFO_STREAM_NAMED("dangerfinder",
                "Could not stop dangerfinder since it was not running in the first place.");
    }
}

// immediately calculate and return results
DangerData DangerFinder::calculateDataNow() {
    ensureRunning();
    calculate();
    std::lock_guard<std::mutex> lock(mutex);
    return mostRecentData;
}

// Gets the most recent results of the dangerfinder thread.
DangerData DangerFinder::getMostRecentData() {
    ensureRunning();
    if (! ranOnce) {
        calculate();
    }
    std::lock_guard<std::mutex> lock(mutex);
    return mostRecentData;
}

// return true if there is a calculated result
bool DangerFinder::hasCalculated() {
    std::lock_guard<std::mutex> lock(mutex);
    return ranOnce;
}

// Load modules to determine dangerscores
void DangerFinder::loadModules() {
    canShootModule = CanShootModule(999);
    distanceModule = DistanceModule(- 1.7/9.0); // factor / max width
    freeModule = FreeModule(0.0);
    hasBallModule = HasBallModule(100);
    orientationModule = OrientationModule(1.7, 3.0, 1);

    dangerModules.push_back(&canShootModule);
    dangerModules.push_back(&distanceModule);
    dangerModules.push_back(&freeModule);
    dangerModules.push_back(&hasBallModule);
    dangerModules.push_back(&orientationModule);
}

} // dangerfinder
} // ai
} // rtt