//
// Created by mrlukasbos on 4-12-18.
//

#include "Input.h"

namespace rtt {
namespace ai {
namespace interface {

std::vector<Drawing> Input::drawings;

// declare static variables
std::map<int, std::vector<std::pair<Vector2, QColor>>> Input::NumTreePoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Input::KeeperPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Input::InterceptPoints;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Input::AttackerPoints;
std::vector<std::pair<Vector2, QColor>> Input::OffensivePoints;

std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> Input::testLines;
std::vector<std::pair<Vector2,QColor>> Input::testPoints;
std::vector<std::pair<Vector2, QColor>> Input::drawP;
std::vector<std::tuple<Vector2, Vector2, QColor>> Input::drawL;

std::mutex Input::keeperMutex;
std::mutex Input::goToPosMutex;
std::mutex Input::interceptMutex;
std::mutex Input::testLineMutex;
std::mutex Input::testPointMutex;
std::mutex Input::offensiveMutex;
std::mutex Input::attackerMutex;
std::mutex Input::drawMutex;
std::mutex Input::drawLinesMutex;


    std::mutex Input::drawingMutex;

void Input::setNumTreePoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    //GoToPosLuThPoints.erase(id); //Probably not needed?
    NumTreePoints[id] = std::move(points);
}

void Input::addNumTreePoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    GTPPoints oldPoints = NumTreePoints[id];
    oldPoints.insert(oldPoints.end(), points.begin(), points.end());
    NumTreePoints[id] = oldPoints;
}

Input::GTPPoints Input::getNumTreePoints(int id) {
    std::lock_guard<std::mutex> lock(goToPosMutex);

    if (NumTreePoints.find(id) != NumTreePoints.end()) {
        return NumTreePoints.at(id);
    }
    return {};

}

void Input::setKeeperPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    std::pair<int, GTPPoints> pair{id, std::move(points)};

    KeeperPoints.erase(id);
    KeeperPoints.insert(pair);
}

Input::GTPPoints Input::getKeeperPoints(int id) {
    std::lock_guard<std::mutex> lock(keeperMutex);

    if (KeeperPoints.find(id) != KeeperPoints.end()) {
        return KeeperPoints.at(id);
    }
    return {};

}

void Input::setInterceptPoints(int id, GTPPoints points) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    std::pair<int, GTPPoints> pair{id, std::move(points)};

    InterceptPoints.erase(id);
    InterceptPoints.insert(pair);
}

Input::GTPPoints Input::getInterceptPoints(int id) {
    std::lock_guard<std::mutex> lock(interceptMutex);

    if (InterceptPoints.find(id) != InterceptPoints.end()) {
        return InterceptPoints.at(id);
    }
    return {};

}

void Input::setTestLines(std::vector<std::pair<std::pair<rtt::Vector2, rtt::Vector2>, QColor>> lines) {
    std::lock_guard<std::mutex> lock(testLineMutex);
    testLines=lines;
}
std::vector<std::pair<std::pair<Vector2,Vector2>,QColor>> Input::getTestLines() {
    std::lock_guard<std::mutex> lock(testLineMutex);
    return testLines;
}
void Input::setTestPoints(std::vector<std::pair<Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(testPointMutex);
    testPoints=points;
}
std::vector<std::pair<Vector2,QColor>> Input::getTestPoints() {
    std::lock_guard<std::mutex> lock(testPointMutex);
    return testPoints;
}

void Input::addDrawPoint(Vector2 position, QColor color) {
    std::pair<Vector2, QColor> point = {position, color};
    drawPoint(point);
}

void Input::drawPoint(std::pair<Vector2, QColor> point) {
    std::lock_guard<std::mutex> lock(drawMutex);
    drawP.push_back(point);
}

void Input::drawPoints(std::vector<std::pair<Vector2, QColor>> points) {
    for (auto &point : points) {
        drawPoint(point);
    }
}

std::vector<std::pair<Vector2, QColor>> Input::getDrawPoints() {
    return drawP;
}

void Input::drawLine(Vector2 pointA, Vector2 pointB, QColor color) {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    drawL.emplace_back(pointA, pointB, color);
}

std::vector<std::tuple<Vector2, Vector2, QColor>> Input::getDrawLines() {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    return drawL;
}

void Input::clearDrawLines() {
    std::lock_guard<std::mutex> lock(drawLinesMutex);
    drawL = {};
}

void Input::clearDrawPoints() {
    std::lock_guard<std::mutex> lock(drawMutex);
    drawP = {};
}

Input::GTPPoints Input::getAttackerPoints(int id) {
    std::lock_guard<std::mutex> lock(attackerMutex);

    if (AttackerPoints.find(id) != AttackerPoints.end()) {
        return AttackerPoints.at(id);
    }
    return {};

}

void Input::drawData(std::string const &name, std::vector<Vector2> points, QColor color, Drawing::DrawingMethod method,
                     Drawing::Depth depth) {
    Input::makeDrawing(name, Drawing(name, std::move(points), std::move(color), method, depth));
}

void Input::makeDrawing(std::string const &name, Drawing const &drawing) {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings.push_back(drawing);
}

const std::vector<Drawing> &Input::getDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    return drawings;
}

void Input::clearDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings = {};
}

} // interface
} // ai
} // rtt