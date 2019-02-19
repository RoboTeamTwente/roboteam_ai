//
// Created by mrlukasbos on 19-2-19.
//

#include <roboteam_ai/src/utilities/Hungarian.h>
#include <gtest/gtest.h>

TEST(HungarianTest, it_works_properly) {
    std::vector<int> assignments;
    rtt::HungarianAlgorithm alg;
    double cost;

    // test small vector
    // this vector is non-square, so a column -1 should be appended
    vector<vector<double>> EXAMPLE1 = {
        {100,   100,    1},
        {100,   2,      21512},
        {1,     4,      9852},
        {6,     30252,  400}
    };

    cost = alg.Solve(EXAMPLE1, assignments);
    std::vector<int> solution = {2, 1, 0, -1};

    EXPECT_EQ(assignments, solution);
    EXPECT_EQ(cost, 4.0);

    vector<vector<double>> EXAMPLE2 = {
            {46,   15,  46, 97, 31},
            {32,   73,  1,  93, 57},
            {28,   92,  9,  17, 72},
            {21,   63,  50, 44, 88},
            {50,   40,  1,  65, 2}

    };

    cost = alg.Solve(EXAMPLE2, assignments);
    solution = {1, 2, 3, 0, 4};

    EXPECT_EQ(assignments, solution);
    EXPECT_EQ(cost, 56.0);


    // test a huge vector
    vector<vector<double>> EXAMPLE3 = {
        { 3093, 6313, 2221, 7215, 4754, 9879, 9339, 6155, 9437, 9313 },
        { 1046, 4993, 2185, 1897, 7849, 6805, 8304, 1136, 2538, 1797 },
        { 3579, 5801, 5341, 3585, 1052, 8261, 2695, 3258, 5615, 9692 },
        { 5376, 7266, 5885, 7695, 8582, 9580, 7088, 4047, 9485, 4021 },
        { 7822, 5213, 6240, 6786, 448, 5220, 9507, 9988, 8612, 5350 },
        { 6681, 9847, 7390, 6890, 1112, 2271, 1754, 2536, 9789, 8767 },
        { 8959, 3856, 5697, 768, 5609, 8791, 7013, 5238, 2669, 9313 },
        { 6939, 7147, 8229, 655, 4129, 9292, 4939, 3572, 7611, 3117 },
        { 3542, 1612, 4412, 5512, 7845, 161, 5359, 5982, 8933, 9399 },
        { 9070, 7698, 3453, 2066, 7055, 3034, 4896, 9342, 6071, 7482 }
    };

    cost = alg.Solve(EXAMPLE3, assignments);
    solution = {2, 0, 7, 9, 4, 6, 8, 3, 1, 5};

    EXPECT_EQ(assignments, solution);
    EXPECT_EQ(cost, 20718.0);
}