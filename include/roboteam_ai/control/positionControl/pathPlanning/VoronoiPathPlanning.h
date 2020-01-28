//;
// Created by ratoone on 05-11-19.
//

#ifndef RTT_VORONOIPATHPLANNING_H
#define RTT_VORONOIPATHPLANNING_H

#include "Voronoi.h"
#include <queue>
#include <list>
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"
#include "control/ControlUtils.h"
#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control{
struct GraphNode{
    Vector2 nextNodePosition;
    double distance;
};

//black magic vector hashing - taken from the boost hash combine
struct hashPoint{
    size_t operator()(const Vector2& point) const {
        size_t xHash = std::hash<double>()(point.x) + 0x9e3779b9;
        return xHash ^ (std::hash<double>()(point.y) + 0x9e3779b9 + (xHash<<6) + (xHash>>2));
    }
};

/**
 * Path planning algorithm. See method computePath for details.
 */
class VoronoiPathPlanning {
private:
    jcv_diagram voronoiDiagram{};

    CollisionDetector &collisionDetector;

    // the adjacency list is a map from the position of the node to the adjacent nodes
    std::unordered_map<Vector2, std::list<GraphNode>, hashPoint> graphAdjacencyList;

    Vector2 convertFromJcvPoint(jcv_point point);

    void generateGraphFromDiagram();

    std::list<Vector2> generatePathDijkstra(const Vector2& initialPosition, const Vector2& targetPosition);

    void computeDiagram(const Vector2 &robotPosition, const Vector2 &targetPosition);

    void completeGraphWithOriginDestination(const Vector2 &robotPosition, const Vector2 &targetPosition);

public:
    /**
    * The collision detector is provided by the position control. This class was intended
    * to be used only with the PositionControl
    * @param collisionDetector
    */
    explicit VoronoiPathPlanning(CollisionDetector& collisionDetector);

    /**
     * Computes a path using the implemented algorithm. It takes into account the
     * obstacles present in the field. <br><br>
     * VoronoiPathPlanning generates the Voronoi diagram using the robots in the rectangle
     * between the initial and final position. Then it creates a graph with the resulting lines
     * and using Dijkstra, it computes the shortest graph path.
     * @param robotPosition the current robot position
     * @param targetPosition the goal position
     * @return a list of points representing the path
     */
    std::list<Vector2> computePath(const Vector2 &robotPosition, const Vector2 &targetPosition);

    /**
     * Used for testing and debugging purposes
     * @return the graph adjancency list build on top of the Voronoi
     */
    const std::unordered_map<Vector2, std::list<GraphNode>, hashPoint> &getGraphAdjacencyList() const;

};
}

#endif //RTT_VORONOIPATHPLANNING_H
