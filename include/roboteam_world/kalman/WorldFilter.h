#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include <util/RobotFilter.h>
#include <util/BallFilter.h>
#include "roboteam_utils/Position.h"
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_proto/World.pb.h"

namespace world {

//This class is a manager for the different Kalman object classes
    class WorldFilter {
    public:
        WorldFilter();
        void addFrame(const proto::SSL_DetectionFrame &msg);
        proto::World getWorld(double time);
    private:
        void update(double time, bool extrapolateLastStep);
        typedef std::map<int, std::vector<std::unique_ptr<RobotFilter>>> robotMap;
        const std::unique_ptr<RobotFilter>& bestFilter(const std::vector<std::unique_ptr<RobotFilter>> &filters);
        const std::unique_ptr<BallFilter>& bestFilter(const std::vector<std::unique_ptr<BallFilter>> &filters);

        robotMap blueBots;
        robotMap yellowBots;
        std::vector<std::unique_ptr<BallFilter>> balls;

        std::mutex filterMutex;
    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
