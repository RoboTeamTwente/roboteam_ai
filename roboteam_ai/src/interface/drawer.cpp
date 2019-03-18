//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::GoToPosLuThPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::KeeperPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::InterceptPoints;
std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> Drawer::testLines;
std::vector<std::pair<Vector2,QColor>> Drawer::testPoints;

std::mutex Drawer::keeperMutex;
std::mutex Drawer::goToPosMutex;
std::mutex Drawer::interceptMutex;
std::mutex Drawer::testLineMutex;
std::mutex Drawer::testPointMutex;

void Drawer::setGoToPosLuThPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    //GoToPosLuThPoints.erase(id); //Probably not needed?
    GoToPosLuThPoints[id] = std::move(points);
}

void Drawer::addGoToPosLuThPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    GTPPoints oldPoints = GoToPosLuThPoints[id];
    oldPoints.insert(oldPoints.end(), points.begin(), points.end());
    GoToPosLuThPoints[id] = oldPoints;
}

Drawer::GTPPoints Drawer::getGoToPosLuThPoints(int id) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints.at(id);
    }
    return {};

}

void Drawer::setKeeperPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    std::pair<int, GTPPoints> pair{id, std::move(points)};

    KeeperPoints.erase(id);
    KeeperPoints.insert(pair);
}

Drawer::GTPPoints Drawer::getKeeperPoints(int id) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    if (KeeperPoints.find(id) != KeeperPoints.end()) {
        return KeeperPoints.at(id);
    }
    return {};

}

void Drawer::setInterceptPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    std::pair<int, GTPPoints> pair{id, std::move(points)};

    InterceptPoints.erase(id);
    InterceptPoints.insert(pair);
}

Drawer::GTPPoints Drawer::getInterceptPoints(int id) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    if (InterceptPoints.find(id) != InterceptPoints.end()) {
        return InterceptPoints.at(id);
    }
    return {};

}
void Drawer::setTestLines(std::vector<std::pair<std::pair<rtt::Vector2, rtt::Vector2>, QColor>> lines) {
    std::lock_guard<std::mutex> lock(testLineMutex);
    testLines=lines;
}
std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> Drawer::getTestLines() {
    std::lock_guard<std::mutex> lock(testLineMutex);
    return testLines;
}
void Drawer::setTestPoints(std::vector<std::pair<Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(testPointMutex);
    testPoints=points;
}
std::vector<std::pair<Vector2,QColor>> Drawer::getTestPoints() {
    std::lock_guard<std::mutex> lock(testPointMutex);
    return testPoints;
}
} // interface
} // ai
} // rtt