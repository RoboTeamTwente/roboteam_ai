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
#include "armadillo"
#include <vector>

namespace rtt {

    TEST(KalmanTest, Armadillo){

        arma::mat::fixed<2,2> A = {{1,2},
                       {3,4}};
        arma::mat::fixed<2,2> B = {{1,2},
                       {3,4}};

        arma::mat::fixed<2,2> C = A * B;
        ASSERT_FLOAT_EQ(C(0, 0), 7);
        ASSERT_FLOAT_EQ(C(0, 1), 10);
        ASSERT_FLOAT_EQ(C(1, 0), 15);
        ASSERT_FLOAT_EQ(C(1, 1), 22);
        A = C - B;
        ASSERT_FLOAT_EQ(A(0, 0), 6);
        ASSERT_FLOAT_EQ(A(0, 1), 8);
        ASSERT_FLOAT_EQ(A(1, 0), 12);
        ASSERT_FLOAT_EQ(A(1, 1), 18);
        A = A + B;
        ASSERT_FLOAT_EQ(A(0, 0), C(0,0));
        ASSERT_FLOAT_EQ(A(0, 1), C(0,1));
        ASSERT_FLOAT_EQ(A(1, 0), C(1,0));
        ASSERT_FLOAT_EQ(A(1, 1), C(1,1));
        arma::mat::fixed<2,2> D = C.t();
        ASSERT_FLOAT_EQ(D(0, 0), 7);
        ASSERT_FLOAT_EQ(D(0, 1), 15);
        ASSERT_FLOAT_EQ(D(1, 0), 10);
        ASSERT_FLOAT_EQ(D(1, 1), 22);
        D = D.i();
        ASSERT_FLOAT_EQ(D(0, 0), 5.5);
        ASSERT_FLOAT_EQ(D(0, 1), -3.75);
        ASSERT_FLOAT_EQ(D(1, 0), -2.5);
        ASSERT_FLOAT_EQ(D(1, 1), 1.75);
        C.zeros();
        ASSERT_FLOAT_EQ(C(0, 0), 0);
        ASSERT_FLOAT_EQ(C(0, 1), 0);
        ASSERT_FLOAT_EQ(C(1, 0), 0);
        ASSERT_FLOAT_EQ(C(1, 1), 0);
        C.eye();
        ASSERT_FLOAT_EQ(C(0, 0), 1);
        ASSERT_FLOAT_EQ(C(0, 1), 0);
        ASSERT_FLOAT_EQ(C(1, 0), 0);
        ASSERT_FLOAT_EQ(C(1, 1), 1);

    }


    TEST(KalmanTest, K) {

        //kalmanInit();

        //setZ(1, 1.0, 1.0, 1.0, 1.0);
        //for (int j = 0; j < 1000; ++j) {
          //  kalmanUpdate();
        //}

        //Position testX = getStates(1);
        //ASSERT_FLOAT_EQ(testX.x, 0);

        //float testK = getK(1);
        //ASSERT_FLOAT_EQ(testK, 0);

    }
}