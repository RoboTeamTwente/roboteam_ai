#ifndef ROBOTEAM_AI_DANGERFINDER_H
#define ROBOTEAM_AI_DANGERFINDER_H

#include <mutex>
#include <thread>
#include <vector>
#include <map>
#include "modules/DangerModule.h"
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "DangerData.h"
#include "ros/ros.h"
#include "modules/CanShootModule.h"
#include "modules/DistanceModule.h"
#include "modules/FreeModule.h"
#include "modules/HasBallModule.h"
#include "modules/OrientationModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class DangerFinder {
    public:
        DangerFinder(const DangerFinder &) = delete;  // It's a singleton; don't copy it.
        void operator=(const DangerFinder &) = delete;
        DangerData getMostRecentData();
        DangerData calculateDataNow();
        void start(int iterationsPerSecond = 20);
        void stop();
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
        std::vector<DangerModule*> dangerModules;
        void calculate();
        void loop(unsigned delayMillis);
        DangerFinder();
        CanShootModule canShootModule;
        DistanceModule distanceModule; // factor / max width
        FreeModule freeModule;
        HasBallModule hasBallModule;
        OrientationModule orientationModule;
};
} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DANGERFINDER_H