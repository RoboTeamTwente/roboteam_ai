//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H


#include "kalmanObject.h"
#include "kalmanUs.h"
#include "kalmanThem.h"
#include "kalmanBall.h"
#include "roboteam_utils/Position.h"
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_proto/World.pb.h"

namespace rtt {

    //This class is a manager for the different Kalman object classes
    class kalmanFilter {
        private:
            double lastFrameTime;

    public:
        kalmanFilter();

        void kalmanUpdate();

        void newFrame(const roboteam_proto::SSL_DetectionFrame& msg);

        roboteam_proto::World getWorld();
        std::string getWorldMsg();

        kalmanThem theirBots[BOTCOUNT];
        kalmanUs ourBots[BOTCOUNT];
        kalmanBall ball;

        std::mutex filterMutex;
    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
