//
// Created by ratoone on 07-11-19.
//

#include <gtest/gtest.h>
#include <control/positionControl/VoronoiPathPlanning.h>
#include <world/Robot.h>
#include <control/ControlUtils.h>

using namespace rtt::ai::world;

TEST(VoronoiPathPlanningTest,computeGraphSuccessfully) {
    int NUMBER_OF_ROBOTS = 4;
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < NUMBER_OF_ROBOTS; i++){
        robots.push_back(std::make_shared<Robot>());
    }
    std::vector<Vector2*> robotPositions(NUMBER_OF_ROBOTS);
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                   [](auto robot)-> Vector2* {return &(robot->pos);});
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robotPositions);
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
    std::vector<Vector2*> robotPositions(NUMBER_OF_ROBOTS);
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                   [](auto robot)-> Vector2* {return &(robot->pos);});
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robotPositions);
    robots[0]->pos = {9, 2};
    robots[1]->pos = {1, 9};
    robots[2]->pos = {1, 3};
    robots[3]->pos = {5, 5};
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 4);
    ASSERT_EQ(*std::next(pathPoints.begin(),1), rtt::Vector2(4.75,0.5));
}

TEST(VoronoiPathPlanningTest,computeDirectPathSuccessfully) {
    std::vector<Vector2*> robots;
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robots);
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 1);
}

TEST(VoronoiPathPlanningTest,avoidSingleRobotSuccessfully) {
    int NUMBER_OF_ROBOTS = 1;
    std::vector<std::shared_ptr<Robot>> robots;
    for (int i = 0; i < NUMBER_OF_ROBOTS; i++){
        robots.push_back(std::make_shared<Robot>());
    }
    std::vector<Vector2*> robotPositions(NUMBER_OF_ROBOTS);
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                   [](auto robot)-> Vector2* {return &(robot->pos);});
    VoronoiPathPlanning posControl = VoronoiPathPlanning(100, 100, robotPositions);
    robots[0]->pos = {5, 5};
    auto pathPoints = posControl.computePath({0,0},{10,10});
    ASSERT_EQ(pathPoints.size(), 2);
}