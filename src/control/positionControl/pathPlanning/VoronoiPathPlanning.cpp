//
// Created by ratoone on 05-11-19.
//

#include "control/positionControl/pathPlanning/VoronoiPathPlanning.h"

namespace rtt::ai::control {

VoronoiPathPlanning::VoronoiPathPlanning(CollisionDetector& collisionDetector) : collisionDetector(collisionDetector) {}

void VoronoiPathPlanning::computeDiagram(const Vector2& robotPosition, const Vector2& targetPosition) {
    auto robots = collisionDetector.getRobotPositions();
    std::vector<jcv_point> robotPositions(robots.size());
    std::transform(robots.begin(), robots.end(), robotPositions.begin(), [](auto robot) -> jcv_point { return {(float)robot.x, (float)robot.y}; });
    jcv_rect playArea = {(jcv_point){(float)std::min(robotPosition.x, targetPosition.x), (float)std::min(robotPosition.y, targetPosition.y)},
                         (jcv_point){(float)std::max(robotPosition.x, targetPosition.x), (float)std::max(robotPosition.y, targetPosition.y)}};
    jcv_diagram_generate(robots.size(), robotPositions.data(), &playArea, nullptr, &voronoiDiagram);
}

void VoronoiPathPlanning::generateGraphFromDiagram() {
    graphAdjacencyList.clear();
    for (const jcv_edge* edgeIterator = jcv_diagram_get_edges(&voronoiDiagram); edgeIterator != nullptr; edgeIterator = edgeIterator->next) {
        // ignore edges of the diagram
        if (edgeIterator->a == 0 && edgeIterator->b == 0 && edgeIterator->c == 0) {
            continue;
        }

        double distance = Vector2(edgeIterator->pos[0].x - edgeIterator->pos[1].x, edgeIterator->pos[0].y - edgeIterator->pos[1].y).length();

        // ignore degenerate lines
        if (distance == 0) {
            continue;
        }

        Vector2 firstEdgePoint = convertFromJcvPoint(edgeIterator->pos[0]);
        Vector2 secondEdgePoint = convertFromJcvPoint(edgeIterator->pos[1]);

        // if the vertex is a new one (not found in the map), add it
        if (graphAdjacencyList.empty() || graphAdjacencyList.find(firstEdgePoint) == graphAdjacencyList.end()) {
            graphAdjacencyList[firstEdgePoint] = std::list<GraphNode>(1, (GraphNode){secondEdgePoint, distance});
        }
        // else append the adjacency list with the new neighbouring site and the new node connection
        else {
            graphAdjacencyList[firstEdgePoint].push_back((GraphNode){secondEdgePoint, distance});
        }

        // repeat for the second node
        if (graphAdjacencyList.empty() || graphAdjacencyList.find(secondEdgePoint) == graphAdjacencyList.end()) {
            graphAdjacencyList[secondEdgePoint] = std::list<GraphNode>(1, (GraphNode){firstEdgePoint, distance});
        } else {
            graphAdjacencyList[secondEdgePoint].push_back((GraphNode){firstEdgePoint, distance});
        }
    }
}

void VoronoiPathPlanning::addPointToGraph(const Vector2& pointToAdd) {
    // 1 or fewer obstacles in the area - no points in the graph
    if (graphAdjacencyList.empty()) {
        graphAdjacencyList[pointToAdd] = {};
        return;
    }

    double minDist = -1;
    Vector2 closestPoint;
    // add point to the graph, connecting it to the closest node
    for (auto const& positionIterator : graphAdjacencyList) {
        if (minDist < 0 || (positionIterator.first - pointToAdd).length() < minDist) {
            closestPoint = positionIterator.first;
            minDist = (positionIterator.first - pointToAdd).length();
        }
    }
    if (graphAdjacencyList.find(pointToAdd) == graphAdjacencyList.end()) {
        graphAdjacencyList[pointToAdd] = std::list<GraphNode>(1, (GraphNode){closestPoint, minDist});
        graphAdjacencyList[closestPoint].push_back({pointToAdd, minDist});
    }
}

std::vector<Vector2> VoronoiPathPlanning::generatePathDijkstra(const Vector2& initialPosition, const Vector2& targetPosition) {
    std::unordered_map<Vector2, float> distanceVector;
    std::unordered_map<Vector2, Vector2> parentVector;
    distanceVector[initialPosition] = 0;
    std::list<Vector2> nodeQueue;
    nodeQueue.push_front(initialPosition);
    for (const Vector2& currentNode : nodeQueue) {
        if (currentNode == targetPosition) {
            break;
        }
        // for each node in the queue, check its neighbors
        for (const GraphNode& adjacentNode : graphAdjacencyList[currentNode]) {
            if (distanceVector.find(adjacentNode.nextNodePosition) == distanceVector.end()) {
                // check if it's already visited or queued
                if (std::find(nodeQueue.begin(), nodeQueue.end(), adjacentNode.nextNodePosition) == nodeQueue.end()) {
                    nodeQueue.push_back(adjacentNode.nextNodePosition);
                }
                distanceVector[adjacentNode.nextNodePosition] = distanceVector[currentNode] + adjacentNode.distance;
                parentVector[adjacentNode.nextNodePosition] = currentNode;
                continue;
            }
            // if the current node's traversal causes a shorter path to get to adjacentNode, update its parent and cost
            if (distanceVector[adjacentNode.nextNodePosition] > distanceVector[currentNode] + adjacentNode.distance) {
                distanceVector[adjacentNode.nextNodePosition] = distanceVector[currentNode] + adjacentNode.distance;
                parentVector[adjacentNode.nextNodePosition] = currentNode;
            }
        }
    }

    std::vector<Vector2> pathPoints;
    for (Vector2 backtrack = targetPosition; parentVector.find(backtrack) != parentVector.end(); backtrack = parentVector[backtrack]) {
        pathPoints.push_back(backtrack);
    }

    // as older points get pushed back, the vector has to be reversed
    std::reverse(pathPoints.begin(), pathPoints.end());
    return pathPoints;
}

std::vector<Vector2> VoronoiPathPlanning::computePath(const Vector2& robotPosition, const Vector2& targetPosition) {
    computeDiagram(robotPosition, targetPosition);
    generateGraphFromDiagram();
    // TODO: avoid one obstacle in voronoi
    addPointToGraph(robotPosition);
    addPointToGraph(targetPosition);
    return generatePathDijkstra(robotPosition, targetPosition);
}

Vector2 VoronoiPathPlanning::convertFromJcvPoint(jcv_point point) {
    // rounded to 3 decimals for testing purposes - 1mm precision is enough
    return Vector2(std::round(1000.0 * point.x) / 1000, std::round(1000.0 * point.y) / 1000);
}

}  // namespace rtt::ai::control