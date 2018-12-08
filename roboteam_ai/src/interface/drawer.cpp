//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::GoToPosLuThPoints;

void Drawer::setGoToPosLuThPoints(int id, std::vector<std::pair<rtt::Vector2, QColor>> points) {

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    GoToPosLuThPoints.erase(id);
    Drawer::GoToPosLuThPoints.insert(pair);
}

std::vector<std::pair<Vector2, QColor>> Drawer::getGoToPosLuThPoints(int id) {

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints[id];
    }
    return {};
}

}
}
}