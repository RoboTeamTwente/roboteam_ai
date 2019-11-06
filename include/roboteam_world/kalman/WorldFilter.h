//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include <util/RobotFilter.h>
#include "KalmanObject.h"
#include "KalmanBall.h"
#include "roboteam_utils/Position.h"
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_proto/World.pb.h"

namespace world {

//This class is a manager for the different Kalman object classes
    class WorldFilter {
    public:
        WorldFilter();
        void kalmanUpdate();
        void addFrame(const proto::SSL_DetectionFrame &msg);
        proto::World getWorld(double time);
    private:
        void update(double time, bool extrapolateLastStep);
        typedef std::map<int, std::vector<std::shared_ptr<RobotFilter>>> robotMap;
        std::shared_ptr<RobotFilter> bestFilter(std::vector<std::shared_ptr<RobotFilter>> filters);
        robotMap blueBots;
        robotMap yellowBots;
        KalmanBall ball;

        std::mutex filterMutex;
    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
