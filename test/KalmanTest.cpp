//
// Created by kjhertenberg on 14-5-19.
//

#include <gtest/gtest.h>
#include "roboteam_world/kalman/kalmanFilter.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_world/world/world_dummy.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include <vector>

namespace rtt {

    TEST(KalmanTest, K) {

        kalmanInit();

        setZ(1, 1.0, 1.0, 1.0, 1.0);
        for (int j = 0; j < 1000; ++j) {
            kalmanUpdate();
        }

        //Position testX = getStates(1);
        //ASSERT_FLOAT_EQ(testX.x, 0);

        //float testK = getK(1);
        //ASSERT_FLOAT_EQ(testK, 0);

    }
}