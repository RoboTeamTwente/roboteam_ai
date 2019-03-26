//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::NumTreePoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::KeeperPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::InterceptPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::AttackerPoints;
std::vector<std::pair<Vector2, QColor>> Drawer::OffensivePoints;
std::vector<std::pair<Vector2, QColor>> Drawer::drawP;
std::vector<std::tuple<Vector2, Vector2, QColor>> Drawer::drawL;

std::mutex Drawer::keeperMutex;
std::mutex Drawer::goToPosMutex;
std::mutex Drawer::interceptMutex;
std::mutex Drawer::offensiveMutex;
std::mutex Drawer::attackerMutex;
std::mutex Drawer::drawMutex;
std::mutex Drawer::drawLinesMutex;

void Drawer::setNumTreePoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    //GoToPosLuThPoints.erase(id); //Probably not needed?
    NumTreePoints[id] = std::move(points);
}

void Drawer::addNumTreePoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    GTPPoints oldPoints = NumTreePoints[id];
    oldPoints.insert(oldPoints.end(), points.begin(), points.end());
    NumTreePoints[id] = oldPoints;
}

Drawer::GTPPoints Drawer::getNumTreePoints(int id) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    if (NumTreePoints.find(id) != NumTreePoints.end()) {
        return NumTreePoints.at(id);
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
void Drawer::drawPoint(Vector2 position, QColor color) {
    std::pair<Vector2, QColor> point = {position, color};
    drawPoint(point);
}

void Drawer::drawPoint(std::pair<Vector2, QColor> point) {
    std::lock_guard<std::mutex> lock(drawMutex);
    drawP.push_back(point);
}

void Drawer::drawPoints(std::vector<std::pair<Vector2, QColor>> points) {
    for (auto &point : points) {
        drawPoint(point);
    }
}

std::vector<std::pair<Vector2, QColor>> Drawer::getDrawPoints() {
    return drawP;
}

void Drawer::drawLine(Vector2 pointA, Vector2 pointB, QColor color) {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    drawL.emplace_back(pointA, pointB, color);
}

std::vector<std::tuple<Vector2, Vector2, QColor>> Drawer::getDrawLines() {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    return drawL;
}

void Drawer::clearDrawLines() {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    drawL = {};
}

void Drawer::clearDrawPoints() {
    std::lock_guard<std::mutex> lock(drawMutex);
    drawP = {};
}

void Drawer::setAttackerPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(attackerMutex);

    std::pair<int, GTPPoints> pair{id, std::move(points)};

    AttackerPoints.erase(id);
    AttackerPoints.insert(pair);
}

Drawer::GTPPoints Drawer::getAttackerPoints(int id) {
    std::lock_guard<std::mutex> lock(attackerMutex);

    if (AttackerPoints.find(id) != AttackerPoints.end()) {
        return AttackerPoints.at(id);
    }
    return {};

}

void Drawer::setOffensivePoints(GTPPoints points){
    std::lock_guard<std::mutex> lock(offensiveMutex);
    OffensivePoints = std::move(points);
}

Drawer::GTPPoints Drawer::getOffensivePoints(){
    std::lock_guard<std::mutex> lock(offensiveMutex);
    return OffensivePoints;
}

} // interface
} // ai
} // rtt