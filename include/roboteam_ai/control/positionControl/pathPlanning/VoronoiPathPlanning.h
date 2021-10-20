//;
// Created by ratoone on 05-11-19.
//

#ifndef RTT_VORONOIPATHPLANNING_H
#define RTT_VORONOIPATHPLANNING_H

#include <list>
#include <queue>
#include "Voronoi.h"
#include "control/ControlUtils.h"
#include "control/positionControl/CollisionDetector.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {
struct GraphNode {
    Vector2 nextNodePosition;
    double distance;
};

/**
 * Path planning algorithm. See method computePath for details.
 */
class VoronoiPathPlanning {
   private:
    jcv_diagram voronoiDiagram{};

    CollisionDetector &collisionDetector;

    /// the adjacency list is a map from the position of the node to the adjacent nodes
    std::unordered_map<Vector2, std::list<GraphNode>> graphAdjacencyList;

    /// helper function to transport a point structure to a Vector2 object
    Vector2 convertFromJcvPoint(jcv_point point);

    /// generate an unoriented graph from the Voronoi diagram
    void generateGraphFromDiagram();

    /// using Dijkstra's algorithm, find the path between the initial and target points in the graph
    std::vector<Vector2> generatePathDijkstra(const Vector2 &initialPosition, const Vector2 &targetPosition);

    /// computes the Voronoi diagram, taking into account the start and end positions for clipping
    void computeDiagram(const Vector2 &robotPosition, const Vector2 &targetPosition);

    /// add the specified point to the graph
    void addPointToGraph(const Vector2 &pointToAdd);

   public:
    /**
     * The collision detector is provided by the position control. This class was intended
     * to be used only with the PositionControl
     * @param collisionDetector
     */
    explicit VoronoiPathPlanning(CollisionDetector &collisionDetector);

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
    std::vector<Vector2> computePath(const Vector2 &robotPosition, const Vector2 &targetPosition);
};
}  // namespace rtt::ai::control

#endif  // RTT_VORONOIPATHPLANNING_H
