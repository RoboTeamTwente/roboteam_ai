//
// Created by ratoone on 07-11-19.
//

#include <gtest/gtest.h>
#include <control/positionControl/VoronoiPathPlanning.h>

using namespace rtt::ai::world;

TEST(VoronoiPathPlanningTest,computeGraphSuccessfully) {
    int NUMBER_OF_ROBOTS = 4;
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < NUMBER_OF_ROBOTS; i++){
        robots.push_back(std::make_shared<Robot>());
    }
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robots);
    robots[0]->pos = {9, 2};
    robots[1]->pos = {1, 9};
    robots[2]->pos = {1, 3};
    robots[3]->pos = {5, 5};
    posControl.computePath({0,0},{10,10});
    auto graph = posControl.getGraphAdjacencyList();
    ASSERT_EQ(graph.size(), 7);
    ASSERT_FALSE(graph.find(rtt::Vector2(6,10)) == graph.end());
}

TEST(VoronoiPathPlanningTest,computeShortestPathSuccessfully) {
    int NUMBER_OF_ROBOTS = 4;
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < NUMBER_OF_ROBOTS; i++){
        robots.push_back(std::make_shared<Robot>());
    }
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robots);
    robots[0]->pos = {9, 2};
    robots[1]->pos = {1, 9};
    robots[2]->pos = {1, 3};
    robots[3]->pos = {5, 5};
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 5);
    ASSERT_EQ(pathPoints[2], rtt::Vector2(4.75,0.5));
}

TEST(VoronoiPathPlanningTest,computeDirectPathSuccessfully) {
    std::vector<std::shared_ptr<Robot>> robots;
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robots);
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 2);
}

TEST(VoronoiPathPlanningTest,avoidSingleRobotSuccessfully) {
    int NUMBER_OF_ROBOTS = 1;
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < NUMBER_OF_ROBOTS; i++){
        robots.push_back(std::make_shared<Robot>());
    }
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robots);
    robots[0]->pos = {5, 5};
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 3);
}