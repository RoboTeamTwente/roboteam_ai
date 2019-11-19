//;
// Created by ratoone on 05-11-19.
//

#ifndef RTT_VORONOIPATHPLANNING_H
#define RTT_VORONOIPATHPLANNING_H

#include "Voronoi.h"
#include <queue>
#include <list>
#include <roboteam_utils/Vector2.h>
#include <utilities/Constants.h>

struct GraphNode{
    rtt::Vector2 nextNodePosition;
    double distance;
};

//black magic vector hashing - taken from the boost hash combine
struct hashPoint{
    size_t operator()(const rtt::Vector2& point) const {
        size_t xHash = std::hash<double>()(point.x) + 0x9e3779b9;
        return xHash ^ (std::hash<double>()(point.y) + 0x9e3779b9 + (xHash<<6) + (xHash>>2));
    }
};

class VoronoiPathPlanning {
private:
    jcv_diagram voronoiDiagram{};
    double fieldWidth;
    double fieldLength;

    std::vector<rtt::Vector2*> robots;

    // the adjacency list is a map from the position of the node to the adjacent nodes
    std::unordered_map<rtt::Vector2, std::list<GraphNode>, hashPoint> graphAdjacencyList;

    rtt::Vector2 convertFromJcvPoint(jcv_point point);

    double computeDistancePointLine(const rtt::Vector2& point, const rtt::Vector2& linePoint1, const rtt::Vector2& linePoint2);

    void generateGraphFromDiagram();

    std::list<rtt::Vector2> generatePathDijkstra(const rtt::Vector2& initialPosition, const rtt::Vector2& targetPosition);

    void computeDiagram(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition);

    void generateGraph(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition);

public:
    VoronoiPathPlanning() = default;

    VoronoiPathPlanning(double fieldWidth, double fieldLength, const std::vector<rtt::Vector2*> &robotPositions);

    std::list<rtt::Vector2> computePath(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition);

    const std::unordered_map<rtt::Vector2, std::list<GraphNode>, hashPoint> &getGraphAdjacencyList() const;
};


#endif //RTT_VORONOIPATHPLANNING_H
