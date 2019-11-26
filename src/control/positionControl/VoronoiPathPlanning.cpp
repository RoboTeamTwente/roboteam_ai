//
// Created by ratoone on 05-11-19.
//

#include <control/positionControl/VoronoiPathPlanning.h>

VoronoiPathPlanning::VoronoiPathPlanning(double fieldWidth, double fieldLength, const std::vector<rtt::Vector2*> &robotPositions) {
    this->fieldLength = fieldLength;
    this->fieldWidth = fieldWidth;
    this->robots = robotPositions;
}

void VoronoiPathPlanning::computeDiagram(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition) {
    std::vector<jcv_point> robotPositions(robots.size());
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
            [](auto robot)-> jcv_point {return {(float)robot->x, (float)robot->y};});
    jcv_rect playArea = {
            (jcv_point){(float)std::min(robotPosition.x,targetPosition.x),(float)std::min(robotPosition.y,targetPosition.y)},
            (jcv_point){(float)std::max(robotPosition.x,targetPosition.x),(float)std::max(robotPosition.y,targetPosition.y)}
    };
    jcv_diagram_generate(robots.size(), robotPositions.data(), &playArea, nullptr, &voronoiDiagram);
}

void VoronoiPathPlanning::generateGraphFromDiagram(){
    graphAdjacencyList.clear();
    for(const jcv_edge* edgeIterator = jcv_diagram_get_edges(&voronoiDiagram); edgeIterator != nullptr; edgeIterator = edgeIterator->next){
        //ignore edges of the diagram
        if (edgeIterator->a == 0 && edgeIterator->b == 0 && edgeIterator->c == 0){
            continue;
        }

        double distance = rtt::Vector2(
                edgeIterator->pos[0].x-edgeIterator->pos[1].x,
                edgeIterator->pos[0].y-edgeIterator->pos[1].y).length();

        //ignore degenerate lines
        if (distance == 0){
            continue;
        }

        rtt::Vector2 firstEdgePoint = convertFromJcvPoint(edgeIterator->pos[0]);
        rtt::Vector2 secondEdgePoint = convertFromJcvPoint(edgeIterator->pos[1]);

        //if the vertex is a new one (not found in the map), add it
        if(graphAdjacencyList.empty() || graphAdjacencyList.find(firstEdgePoint) == graphAdjacencyList.end()){
            graphAdjacencyList.insert({firstEdgePoint, std::list<GraphNode>(1,(GraphNode){secondEdgePoint, distance})});
        }
        //else append the adjacency list with the new neighbouring site and the new node connection
        else{
            graphAdjacencyList[firstEdgePoint].push_back((GraphNode){secondEdgePoint, distance});
        }

        //repeat for the second node
        if(graphAdjacencyList.empty() || graphAdjacencyList.find(secondEdgePoint) == graphAdjacencyList.end()){
            graphAdjacencyList[secondEdgePoint] = std::list<GraphNode>(1,(GraphNode){firstEdgePoint, distance});
        }
        else{
            graphAdjacencyList[secondEdgePoint].push_back((GraphNode){firstEdgePoint, distance});
        }
    }
}

void VoronoiPathPlanning::completeGraphWithOriginDestination(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition) {
    //1 or fewer obstacles in the area - numsites == 1 or 2
    if (voronoiDiagram.numsites <= 2){
        double distance = (robotPosition - targetPosition).length();
        // no obstacle or the distance to the obstacle safe
        if (voronoiDiagram.numsites < 2 || computeDistancePointLine(
                convertFromJcvPoint(voronoiDiagram.internal->sites->p),robotPosition,targetPosition) > 2 * rtt::ai::Constants::ROBOT_RADIUS()) {
            graphAdjacencyList.insert({robotPosition, std::list<GraphNode>(1, (GraphNode) {targetPosition, distance})});
            graphAdjacencyList.insert({targetPosition, std::list<GraphNode>(1, (GraphNode) {robotPosition, distance})});
            return;
        }
        //TODO: avoid one obstacle in voronoi
        graphAdjacencyList.insert({robotPosition, std::list<GraphNode>(1, (GraphNode) {targetPosition, distance})});
        graphAdjacencyList.insert({targetPosition, std::list<GraphNode>(1, (GraphNode) {robotPosition, distance})});
        return;
    }

    double minOriginDist = -1;
    rtt::Vector2 closestOriginPoint;
    double minTargetDist = -1;
    rtt::Vector2 closestTargetPoint;
    //add origin and target to the graph
    for(auto const& positionIterator : graphAdjacencyList){
        if(minOriginDist < 0 || (positionIterator.first-robotPosition).length() < minOriginDist){
            closestOriginPoint = positionIterator.first;
            minOriginDist = (positionIterator.first-robotPosition).length();
        }
        if(minTargetDist < 0 || (positionIterator.first-targetPosition).length() < minTargetDist){
            closestTargetPoint = positionIterator.first;
            minTargetDist = (positionIterator.first-targetPosition).length();
        }
    }
    if (graphAdjacencyList.find(robotPosition) == graphAdjacencyList.end()) {
        graphAdjacencyList.insert(
                {robotPosition, std::list<GraphNode>(1, (GraphNode) {closestOriginPoint, minOriginDist})});
    }
    if (graphAdjacencyList.find(targetPosition) == graphAdjacencyList.end()) {
        graphAdjacencyList[closestTargetPoint].push_back({targetPosition, minTargetDist});
    }
}

const std::unordered_map<rtt::Vector2, std::list<GraphNode>, hashPoint> &
VoronoiPathPlanning::getGraphAdjacencyList() const {
    return graphAdjacencyList;
}

std::list<rtt::Vector2> VoronoiPathPlanning::generatePathDijkstra(const rtt::Vector2& initialPosition, const rtt::Vector2& targetPosition) {
    std::unordered_map<rtt::Vector2, float, hashPoint> distanceVector;
    std::unordered_map<rtt::Vector2, rtt::Vector2, hashPoint> parentVector;
    distanceVector.insert({initialPosition,0});
    std::list<rtt::Vector2> nodeQueue;
    nodeQueue.push_front(initialPosition);
    for(const rtt::Vector2& currentNode: nodeQueue){
        if (currentNode == targetPosition){
            break;
        }
        for (const GraphNode& adjacentNode : graphAdjacencyList[currentNode]){
            if (distanceVector.find(adjacentNode.nextNodePosition) == distanceVector.end()) {
                if (std::find(nodeQueue.begin(), nodeQueue.end(), adjacentNode.nextNodePosition) == nodeQueue.end()) {
                    nodeQueue.push_back(adjacentNode.nextNodePosition);
                }
                distanceVector.insert({adjacentNode.nextNodePosition, distanceVector[currentNode] + adjacentNode.distance});
                parentVector.insert({adjacentNode.nextNodePosition, currentNode});
                continue;
            }
            if(distanceVector[adjacentNode.nextNodePosition] > distanceVector[currentNode] + adjacentNode.distance){
                distanceVector[adjacentNode.nextNodePosition] = distanceVector[currentNode] + adjacentNode.distance;
                parentVector[adjacentNode.nextNodePosition] = currentNode;
            }
        }
    }

    std::list<rtt::Vector2> pathPoints;
    for (rtt::Vector2 backtrack = targetPosition; parentVector.find(backtrack) != parentVector.end(); backtrack = parentVector[backtrack]){
        pathPoints.push_back(backtrack);
    }
    std::reverse(pathPoints.begin(), pathPoints.end());
    return pathPoints;
}

std::list<rtt::Vector2> VoronoiPathPlanning::computePath(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition) {
    computeDiagram(robotPosition, targetPosition);
    generateGraphFromDiagram();
    completeGraphWithOriginDestination(robotPosition, targetPosition);
    return generatePathDijkstra(robotPosition, targetPosition);
}

rtt::Vector2 VoronoiPathPlanning::convertFromJcvPoint(jcv_point point) {
    return rtt::Vector2(std::round(1000.0*point.x)/1000, std::round(1000.0*point.y)/1000);
}

double VoronoiPathPlanning::computeDistancePointLine(const rtt::Vector2& point, const rtt::Vector2& linePoint1, const rtt::Vector2& linePoint2) {
    return std::abs(
            (linePoint1.y - linePoint2.y) * point.x -
            (linePoint1.x - linePoint2.x) * point.y +
            linePoint1.x * linePoint2.y -
            linePoint1.y * linePoint2.x
    ) / (linePoint1 - linePoint2).length();
}

void VoronoiPathPlanning::setRobotPositions(const std::vector<rtt::Vector2 *> &robots) {
    this->robots = robots;
}
