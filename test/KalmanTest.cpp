//
// Created by kjhertenberg on 14-5-19.
//

#include <gtest/gtest.h>
#include "armadillo"
#include <vector>
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>

namespace world {
    void kalman_dummy_frame(float ballx, float bally, float botx, float boty, float botw, proto::SSL_DetectionFrame * frame) {
        proto::SSL_DetectionRobot * bot = frame->add_robots_yellow();
        bot->set_x(botx);
        bot->set_y(boty);
        bot->set_orientation(botw);

        proto::SSL_DetectionBall * ball = frame->add_balls();
        ball->set_x(ballx);
        ball->set_y(bally);
    }

    TEST(KalmanTest, Armadillo){
        //test to see if library is working

        //Initialisation
        arma::mat::fixed<2,2> A = {{1,2},
                                   {3,4}};
        arma::mat::fixed<2,2> B = {{1,2},
                                   {3,4}};

        //dot product multiplication
        arma::mat::fixed<2,2> C = A * B;
        ASSERT_DOUBLE_EQ(C(0, 0), 7);
        ASSERT_DOUBLE_EQ(C(0, 1), 10);
        ASSERT_DOUBLE_EQ(C(1, 0), 15);
        ASSERT_DOUBLE_EQ(C(1, 1), 22);
        //subtraction
        A = C - B;
        ASSERT_DOUBLE_EQ(A(0, 0), 6);
        ASSERT_DOUBLE_EQ(A(0, 1), 8);
        ASSERT_DOUBLE_EQ(A(1, 0), 12);
        ASSERT_DOUBLE_EQ(A(1, 1), 18);
        //addition
        A = A + B;
        ASSERT_DOUBLE_EQ(A(0, 0), C(0,0));
        ASSERT_DOUBLE_EQ(A(0, 1), C(0,1));
        ASSERT_DOUBLE_EQ(A(1, 0), C(1,0));
        ASSERT_DOUBLE_EQ(A(1, 1), C(1,1));
        //transpose
        arma::mat::fixed<2,3> D = {{1,2,3},
                                   {4,5,6}};
        arma::mat::fixed<3,2> E = D.t();
        ASSERT_DOUBLE_EQ(E(0, 0), 1);
        ASSERT_DOUBLE_EQ(E(0, 1), 4);
        ASSERT_DOUBLE_EQ(E(1, 0), 2);
        ASSERT_DOUBLE_EQ(E(1, 1), 5);
        ASSERT_DOUBLE_EQ(E(2, 0), 3);
        ASSERT_DOUBLE_EQ(E(2, 1), 6);
        //inverse
        arma::mat::fixed<2,2> F = C.i();
        ASSERT_DOUBLE_EQ(F(0, 0), 5.5);
        ASSERT_DOUBLE_EQ(F(0, 1), -2.5);
        ASSERT_DOUBLE_EQ(F(1, 0), -3.75);
        ASSERT_DOUBLE_EQ(F(1, 1), 1.75);
        //zero initialisation
        C.zeros();
        ASSERT_DOUBLE_EQ(C(0, 0), 0);
        ASSERT_DOUBLE_EQ(C(0, 1), 0);
        ASSERT_DOUBLE_EQ(C(1, 0), 0);
        ASSERT_DOUBLE_EQ(C(1, 1), 0);
        //identity initialisation
        C.eye();
        ASSERT_DOUBLE_EQ(C(0, 0), 1);
        ASSERT_DOUBLE_EQ(C(0, 1), 0);
        ASSERT_DOUBLE_EQ(C(1, 0), 0);
        ASSERT_DOUBLE_EQ(C(1, 1), 1);
    }
}