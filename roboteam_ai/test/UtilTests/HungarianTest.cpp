//
// Created by mrlukasbos on 19-2-19.
//

#include <roboteam_ai/src/utilities/Hungarian.h>
#include <gtest/gtest.h>

TEST(HungarianTest, it_works_properly) {

const hungarian::Matrix EXAMPLE1 = {
        {100, 100, 1},
        {100, 2, 21512},
        {1, 4, 9852},
        {6, 30252, 400}
};

const hungarian::Matrix SOLUTION1 = {
        {0, 0, 1, 0},
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1}
};



    hungarian::Result r = hungarian::Solve(EXAMPLE1, hungarian::MODE_MINIMIZE_COST);
    hungarian::PrintMatrix(r.assignment);
    ASSERT_TRUE(r.success);
    ASSERT_EQ(r.assignment, SOLUTION1);


    const hungarian::Matrix EXAMPLE2 = {
            {100, 1},
            {100, 12},
            {1, 4},
            {6, 30252}
    };

// TODO: this isn't right
    const hungarian::Matrix SOLUTION2 = {
            {0,0,1,0},
            {0,1,0,0},
            {1,0,0,0},
            {0,0,0,1}
    };

    r = hungarian::Solve(EXAMPLE1, hungarian::MODE_MINIMIZE_COST);
    hungarian::PrintMatrix(r.assignment);
    ASSERT_TRUE(r.success);
    ASSERT_EQ(r.assignment, SOLUTION2);
}